/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  GoldCorpHF - x6x00-df: direction finding with a KrakenSDR (docs/plans/kraken_df_plan.md, Phase 1). The bare
 *  minimum the plan's section 1.4 lists, and nothing else: no Heimdall, no ZeroMQ, no web page.
 *
 *    - the Kraken found by its serials (1000-1004, CH0-CH4), never by RTL number; opened one at a time, equal manual
 *      gain, every AGC off, bias-T and noise source off;
 *    - one reader per stick (librtlsdr's async read) into a ring of its own, with the time each transfer came in;
 *    - calibration (df_cal.c) on one block of the Kraken's own noise source (GPIO 0 of CH0): the streams aligned in
 *      software (whole samples, first from the transfers' times, then from the noise), the rest of the delay, the
 *      phase and the gain corrected per signal by the engine (df_engine.c), at start, after a retune or a gain
 *      change, on a slip and on a timer;
 *    - the engine on every 0.1 s block: the bearing and how far to trust it, every channel's level and whether it was
 *      used (df_proto.h's DFDO), CH0's spectrum (DFSP), the state (DFST);
 *    - one client on the abstract socket "@x6x00-df" (SOCK_SEQPACKET), the GUI's page I; text commands back.
 *
 *  Receive only: nothing here transmits. The noise source is inside the Kraken and disconnects the antennas while
 *  it is on (so nothing else may listen on a Kraken stick while this runs).
 *
 *    x6x00-df [-F] [-S name] [-p] [-f HZ] [-s RATE] [-g TENTHS] [-t OFFSET,BW] [-a uca|ula,N,MM] [-m METHOD]
 *             [-i SECONDS] [-n CHANNELS] [-D] [-C CORES] [-I BW] [-T BEARING[,DEG_PER_S[,DEAD_CH]]]
 *      -F  in the foreground (else it goes to the background; its log /tmp/x6x00-df.log)
 *      -p  print one line per estimate (with -F: a test from a shell)
 *      -D  leave the tuners' PLL dither on (with our patched librtlsdr it is switched off: test 1g compares)
 *      -I  BW: the IQ server's band in Hz (default 25000: 64 kS/s); 0 no IQ server (ports 5000, 5001)
 *      -C  READERS,ENGINE[,PRIO]: the cores (default 2,3,10: the readers on CPU2 at SCHED_FIFO 10, the engine on CPU3,
 *          the musb interrupt moved to CPU1); -C off leaves all of it to the kernel
 *      -n  open CH0 to CH(n-1) only (2-5; CH0 always, it has the noise source): a bench with another program on the
 *          last stick. The array keeps its positions: UCA-5 with -n 4 is the circle's first four
 *      -T  a synthetic Kraken instead of the sticks: a carrier at BEARING degrees (turning DEG_PER_S), on 6 s
 *          and off 3 s, each channel with its own delay, phase and gain, and the noise source; DEAD_CH's antenna
 *          gets 30 dB less of the signal. For page I without a Kraken, and for tests
 */

#define _GNU_SOURCE
#include "df.h"
#include "df_proto.h"
#include "df_gps.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include <poll.h>

#ifndef DF_NO_RTL
#include <rtl-sdr.h>
/* the PLL's dither switch: in our librtlsdr with buildroot's rtlsdr-blog patch 0002 (KrakenRF's has it too). It is
 * a must: with the dither on, the radio's Kraken's bearing swept the whole circle every 3 s; off, it held within 0.6
 * degrees for 30 s (2026-11-17). A weak symbol, so a library without it still links, and the daemon says so */
extern int rtlsdr_set_dithering(rtlsdr_dev_t *dev, int dither) __attribute__((weak));
#else
typedef struct rtlsdr_dev rtlsdr_dev_t;
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NCH_KRAKEN  5
#define RING_BITS   21
#define RING        (1L << RING_BITS)           /* samples a channel keeps: 2 s at 1.024 MS/s, 0.87 s at 2.4 */
#define BUFS        15                          /* transfers queued per stick */
#define MAXLAG      4096                        /* whole-sample delays looked for around the first guess */
#define SETTLE_S    0.2                         /* after the noise source is switched */
#define HOLD_S      30.0

/* ------------------------------------------------------------------------------------------------ the channels */

typedef struct {
    char                serial[16];
    rtlsdr_dev_t       *dev;
    uint8_t            *ring;                   /* RING samples, I then Q */
    _Atomic int64_t     wr;                     /* samples written */
    _Atomic double      t0;                     /* when its sample 0 came, by the earliest transfers since a reset */
    _Atomic int         t0_reset;
    atomic_bool         ended;                  /* its read ended by itself (pulled out) */
    _Atomic double      last_t;
    pthread_t           th;
    bool                th_on;
} chan_t;

static chan_t   ch[DF_MAXCH];
static int      nch;                            /* streaming */
static int      n_open = NCH_KRAKEN;            /* -n: CH0 to CH(n-1) only (a bench with B on CH4) */
static bool     dither_on;                      /* -D: the tuners' PLL dither left on (test 1g) */
static bool     synth;
static double   synth_bearing = 40, synth_rate = 0;
static int      synth_dead = -1;
static atomic_bool synth_noise;                 /* the synthetic noise source */
static atomic_bool quit_now;
static atomic_bool streaming;

/* settings (the worker's own; commands are queued) */
static uint32_t fc = 433920000, rate = 1024000, bw = 12500;
static int32_t  f_off = 100000;
static int      gain = 280, method = DF_MUSIC, sql = 80, cal_every = 120;
static unsigned user_off;
static df_array_t arr;

/* state for DFST */
static int      st = DF_ST_NODEV, cals, cal_fails, slips, overruns, cpu_pct;
static char     why[64];
static char     note[64];                       /* what stays true while open (channels the USB could not feed) */
static df_cal_t cal;
static bool     have_cal;
static double   cal_at = -1;
static int64_t  kb[DF_MAXCH];                   /* channel m read kb[m] later than CH0 */
static df_est_t last;
static double   last_good_at = -1, last_bearing = -1, last_mirror = -1, last_uncert;
/* What HOLD shows once the signal is under the squelch: the last estimate over it whose fit is worth holding, with its
 * own fit and +-. An estimate taken across a burst's end (half signal, half noise) is over the squelch too, and held as
 * it was it put a wild bearing on screen at a red fit: 198.7 degrees after bursts at 057 (2026-11-17, page I at 915
 * MHz). With none such since the signal came, the last one over the squelch, as before */
#define HOLD_FIT    0.50
static double   held_bearing = -1, held_mirror = -1, held_uncert, held_fit;
static bool     slip_shown;

static double now_s(void) {
    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec + t.tv_nsec * 1e-9;
}

static void logf_(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void logf_(const char *fmt, ...) {
    va_list ap;
    time_t  t = time(NULL);
    char    ts[16];

    strftime(ts, sizeof(ts), "%H:%M:%S", localtime(&t));
    fprintf(stderr, "%s df: ", ts);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

/* a transfer in: into the ring, its time noted */
static void put(chan_t *c, const uint8_t *buf, uint32_t len) {
    int64_t w = atomic_load(&c->wr), n = len / 2;
    size_t  at = (size_t)(w & (RING - 1)), first = (size_t)(RING - (int64_t)at) < (size_t)n ? (size_t)(RING - at) : (size_t)n;
    double  t = now_s();

    memcpy(c->ring + 2 * at, buf, 2 * first);
    if ((int64_t)first < n) memcpy(c->ring, buf + 2 * first, 2 * ((size_t)n - first));
    atomic_store(&c->wr, w + n);
    atomic_store(&c->last_t, t);

    double est = t - (double)(w + n) / rate;   /* its sample 0's time if this transfer had no delay at all */

    if (atomic_exchange(&c->t0_reset, 0) || est < atomic_load(&c->t0)) atomic_store(&c->t0, est);
}

/* ------------------------------------------------------------------------------------------------ gpsd (DFGP) */

/* gpsd when it runs (ROADMAP.md, 4): its reports read on a thread of their own, the heading and the fix sent to the
 * GUI once a second. DF_GPSD=host:port for another gpsd (default 127.0.0.1:2947); tried again every 5 s while it is
 * not there. Nothing here is needed: with no gpsd the bearings keep the heading set by hand */
static pthread_mutex_t gps_lock = PTHREAD_MUTEX_INITIALIZER;
static df_gps_t        gps;
static atomic_bool     gps_up;

static void logf_(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static double now_s(void);

static void *gps_thread(void *arg) {
    const char *e = getenv("DF_GPSD");
    char        host[64] = "127.0.0.1";
    int         port = 2947;
    bool        said = false;

    (void)arg;
    if (e) sscanf(e, "%63[^:]:%d", host, &port);
    while (!atomic_load(&quit_now)) {
        struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port)};
        int                fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        char               buf[4096];
        size_t             have = 0;
        static const char  watch[] = "?WATCH={\"enable\":true,\"json\":true}\n";

        if (fd < 0 || inet_pton(AF_INET, host, &a.sin_addr) != 1 || connect(fd, (struct sockaddr *)&a, sizeof(a)) ||
            write(fd, watch, sizeof(watch) - 1) != (ssize_t)(sizeof(watch) - 1)) {
            if (fd >= 0) close(fd);
            for (int i = 0; i < 50 && !atomic_load(&quit_now); i++) usleep(100000);
            continue;
        }
        atomic_store(&gps_up, true);
        if (!said) logf_("gpsd: reading %s:%d", host, port), said = true;
        while (!atomic_load(&quit_now)) {
            struct pollfd p = {.fd = fd, .events = POLLIN};
            int           r = poll(&p, 1, 1000);

            if (r == 0) continue;
            ssize_t n = r > 0 ? read(fd, buf + have, sizeof(buf) - 1 - have) : -1;
            if (n <= 0) break;
            have += (size_t)n;
            buf[have] = 0;
            char *line = buf, *nl;
            while ((nl = strchr(line, '\n'))) {
                *nl = 0;
                pthread_mutex_lock(&gps_lock);
                df_gps_line(&gps, line, now_s());
                pthread_mutex_unlock(&gps_lock);
                line = nl + 1;
            }
            have = strlen(line);
            memmove(buf, line, have);
            if (have >= sizeof(buf) - 1) have = 0;         /* a line too long for us: let go of it */
        }
        close(fd);
        atomic_store(&gps_up, false);
    }
    return NULL;
}

static void send_msg(const void *m, size_t n);

static void send_gps(void) {
    df_gp_msg_t m;
    double      t = now_s(), deg = 0, lat = 0, lon = 0;

    memset(&m, 0, sizeof(m));
    memcpy(m.tag, "DFGP", 4);
    pthread_mutex_lock(&gps_lock);
    int  src = df_gps_heading(&gps, t, &deg);
    bool fix = df_gps_fix(&gps, t, &lat, &lon);
    int  mode = gps.mode;
    double speed = gps.speed;
    pthread_mutex_unlock(&gps_lock);
    m.gpsd = atomic_load(&gps_up);
    m.mode = (uint8_t)(fix ? mode : 0);
    m.head_src = (uint8_t)src;
    m.head_ddeg = (int16_t)(src ? lrint(deg * 10) % 3600 : -1);
    m.speed_cms = speed < 0 ? 0xffff : (uint16_t)fmin(65534, speed * 100);
    m.lat_e7 = fix ? (int32_t)llround(lat * 1e7) : 0;
    m.lon_e7 = fix ? (int32_t)llround(lon * 1e7) : 0;
    send_msg(&m, sizeof(m));
}

/* --------------------------------------------------------------------------------------------- the cores (-C) */

/* Where the work runs on the radio's four Cortex-A7 cores (ROADMAP.md, 1). Every interrupt lands on CPU0 by default,
 * the musb host's PIO copies (25,000 a second with five sticks) among the base's serial stream, its I2C and the
 * WiFi's: so the musb interrupt goes to CPU1, the sticks' readers to CPU2 at a modest real-time priority (a reader that
 * waits too long is what makes a stick lose samples), the engine to CPU3 as a normal thread (behind, it overruns its
 * ring and says so). Never Heimdall's priority 99 with the real-time throttle off: the GUI must keep feeding the base
 * every second. -C READERS,ENGINE,PRIO (default 2,3,10), -C off: none of it. Nothing is fatal: a step that fails is
 * logged once and the rest goes on */
static int cpu_readers = 2, cpu_engine = 3, rt_prio = 10;
static bool cores_on = true;

static void logf_(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void pin_self(int cpu, int prio, const char *who) {
    cpu_set_t set;

    if (!cores_on || cpu < 0 || cpu >= CPU_SETSIZE || cpu >= sysconf(_SC_NPROCESSORS_ONLN)) return;
    CPU_ZERO(&set);
    CPU_SET(cpu, &set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set)) logf_("%s: not pinned to CPU%d", who, cpu);
    if (prio > 0) {
        struct sched_param sp = {.sched_priority = prio};

        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
            logf_("%s: no real-time priority (%s)", who, strerror(errno));
    }
}

/* The musb host's interrupt on CPU1 (a mask of 2), from /proc/interrupts' "musb-hdrc" line; said once */
static void musb_irq_away(void) {
    FILE *f = fopen("/proc/interrupts", "r");
    char  l[512], path[64];
    int   irq = -1;

    if (!cores_on || !f) {
        if (f) fclose(f);
        return;
    }
    while (fgets(l, sizeof(l), f))
        if (strstr(l, "musb-hdrc") && sscanf(l, " %d:", &irq) == 1) break;
    fclose(f);
    if (irq < 0) return;
    snprintf(path, sizeof(path), "/proc/irq/%d/smp_affinity", irq);
    if ((f = fopen(path, "w"))) {
        bool ok = fputs("2", f) >= 0;

        ok = fclose(f) == 0 && ok;
        logf_("the musb interrupt (%d) %s CPU1", irq, ok ? "on" : "could not be moved to");
    }
}

/* ------------------------------------------------------------------------------------------- the real Kraken */

#ifndef DF_NO_RTL
static void on_read(unsigned char *buf, uint32_t len, void *ctx) {
    chan_t *c = ctx;

    if (!atomic_load(&streaming)) {
        rtlsdr_cancel_async(c->dev);
        return;
    }
    put(c, buf, len);
}

/* 64 KiB transfers (DF_XFER): 15 of them a stick, 4.8 MB for five (usbfs_memory_mb is 16) */
static void *reader(void *arg) {
    chan_t     *c = arg;
    const char *e = getenv("DF_XFER");
    uint32_t    len = e ? (uint32_t)atoi(e) & ~511u : 65536;

    if (len < 4096) len = 65536;

    pin_self(cpu_readers, rt_prio, "a reader");
    rtlsdr_reset_buffer(c->dev);
    rtlsdr_read_async(c->dev, on_read, c, BUFS, len);
    if (atomic_load(&streaming)) atomic_store(&c->ended, true);
    return NULL;
}

static int nearest_gain(rtlsdr_dev_t *d, int tenths) {
    int g[64], n = rtlsdr_get_tuner_gains(d, NULL), best = 0;

    if (n <= 0 || n > 64) return tenths;
    rtlsdr_get_tuner_gains(d, g);
    for (int i = 1; i < n; i++)
        if (abs(g[i] - tenths) < abs(g[best] - tenths)) best = i;
    return g[best];
}

static bool read_line(const char *path, char *b, size_t n) {
    FILE *f = fopen(path, "r");
    bool  ok = f && fgets(b, (int)n, f);

    if (f) fclose(f);
    if (ok) b[strcspn(b, "\r\n")] = 0;
    return ok;
}

/* the five serials on USB? From sysfs: no USB traffic to sticks another program may hold */
static bool kraken_there(void) {
    DIR           *d = opendir("/sys/bus/usb/devices");
    struct dirent *e;
    unsigned       got = 0;

    while (d && (e = readdir(d))) {
        char p[300], v[8], pr[8], sn[32];

        if (e->d_name[0] == '.' || strchr(e->d_name, ':')) continue;
        snprintf(p, sizeof(p), "/sys/bus/usb/devices/%s/idVendor", e->d_name);
        if (!read_line(p, v, sizeof(v)) || strcmp(v, "0bda")) continue;
        snprintf(p, sizeof(p), "/sys/bus/usb/devices/%s/idProduct", e->d_name);
        if (!read_line(p, pr, sizeof(pr)) || (strcmp(pr, "2838") && strcmp(pr, "2832"))) continue;
        snprintf(p, sizeof(p), "/sys/bus/usb/devices/%s/serial", e->d_name);
        if (!read_line(p, sn, sizeof(sn))) continue;
        int k = atoi(sn) - 1000;

        if (k >= 0 && k < NCH_KRAKEN && strlen(sn) == 4) got |= 1u << k;
    }
    if (d) closedir(d);
    if (got != (1u << NCH_KRAKEN) - 1) {        /* say which are missing (page I shows it) */
        int n = snprintf(why, sizeof(why), "waiting for the Kraken's sticks:");

        for (int k = 0; k < NCH_KRAKEN && n < (int)sizeof(why) - 6; k++)
            if (!(got >> k & 1)) n += snprintf(why + n, sizeof(why) - (size_t)n, " %d", 1000 + k);
        return false;
    }
    return true;
}

static void hw_close(void) {
    atomic_store(&streaming, false);
    for (int m = 0; m < nch; m++)
        if (ch[m].dev) rtlsdr_cancel_async(ch[m].dev);
    for (int m = 0; m < nch; m++)
        if (ch[m].th_on) pthread_join(ch[m].th, NULL), ch[m].th_on = false;
    if (nch && ch[0].dev) rtlsdr_set_bias_tee_gpio(ch[0].dev, 0, 0);     /* the noise source off */
    for (int m = 0; m < nch; m++)
        if (ch[m].dev) rtlsdr_close(ch[m].dev), ch[m].dev = NULL;
}

/* Each stick opened, set up and started before the next (as five rtl_test started 2 s apart all streamed; four
 * opened first and started 0.3 or 1 s apart left the fourth with nothing, 2026-11-17), DF_STAGGER_MS apart (1000) */
static int hw_open(void) {
    const char *e = getenv("DF_STAGGER_MS");
    int         stagger = e ? atoi(e) : 1000;

    nch = 0;
    atomic_store(&streaming, true);
    for (int m = 0; m < n_open; m++) {
        chan_t *c = &ch[m];
        int     idx;

        snprintf(c->serial, sizeof(c->serial), "%d", 1000 + m);
        if ((idx = rtlsdr_get_index_by_serial(c->serial)) < 0 || rtlsdr_open(&c->dev, (uint32_t)idx) != 0) {
            c->dev = NULL;
            if (m >= 3) {                       /* CH0 and two more: a bearing from those, the rest GONE */
                snprintf(note, sizeof(note), "CH%d-%d would not open (another program has them)", m, n_open - 1);
                logf_("%s", note);
                break;
            }
            snprintf(why, sizeof(why), "CH%d (serial %.6s) would not open: held by another program?", m, c->serial);
            hw_close();
            nch = 0;
            return -1;
        }
        nch = m + 1;
        rtlsdr_set_bias_tee(c->dev, 0);         /* GPIO 0: on CH0 the noise source, off */
        rtlsdr_set_sample_rate(c->dev, rate);
        rtlsdr_set_center_freq(c->dev, fc);
        rtlsdr_set_tuner_gain_mode(c->dev, 1);
        rtlsdr_set_tuner_gain(c->dev, nearest_gain(c->dev, gain));
        rtlsdr_set_agc_mode(c->dev, 0);
        if (rtlsdr_set_dithering && !dither_on) rtlsdr_set_dithering(c->dev, 0);    /* retunes to fc */
        if (getenv("DF_TESTMODE")) rtlsdr_set_testmode(c->dev, 1);                  /* a bench check: the counter */
        atomic_store(&c->wr, 0);
        atomic_store(&c->ended, false);
        atomic_store(&c->t0_reset, 1);
        atomic_store(&c->last_t, now_s() + stagger / 1000.0 * (n_open - m));     /* not "stopped" while the rest open */
        c->th_on = pthread_create(&c->th, NULL, reader, c) == 0;
        if (m + 1 < n_open) usleep((useconds_t)stagger * 1000);
    }
    gain = nearest_gain(ch[0].dev, gain);
    if (!rtlsdr_set_dithering && !note[0])
        snprintf(note, sizeof(note), "librtlsdr without the dither switch: bearings will wander");
    logf_("Kraken open: %u Hz, %u S/s, gain %.1f dB, PLL dither %s", fc, rate, gain / 10.0,
          !rtlsdr_set_dithering ? "as the library leaves it (no switch)" : dither_on ? "on" : "off");
    return 0;
}

static void hw_noise(bool on) {
    if (nch && ch[0].dev) rtlsdr_set_bias_tee_gpio(ch[0].dev, 0, on);
}

/* channels from k on let go (they streamed nothing): their reads cancelled, closed */
static void hw_drop_from(int k) {
    for (int m = k; m < nch; m++)
        if (ch[m].dev) rtlsdr_cancel_async(ch[m].dev);
    for (int m = k; m < nch; m++)
        if (ch[m].th_on) pthread_join(ch[m].th, NULL), ch[m].th_on = false;
    for (int m = k; m < nch; m++)
        if (ch[m].dev) rtlsdr_close(ch[m].dev), ch[m].dev = NULL;
    nch = k;
}

static void hw_tune(void) {
    for (int m = 0; m < nch; m++) rtlsdr_set_center_freq(ch[m].dev, fc);
}

static void hw_gain(void) {
    gain = nearest_gain(ch[0].dev, gain);
    for (int m = 0; m < nch; m++) rtlsdr_set_tuner_gain(ch[m].dev, gain);
}
#else
static void hw_drop_from(int k) { (void)k; }
static bool kraken_there(void) { return false; }
static void hw_close(void) {}
static int  hw_open(void) { return -1; }
static void hw_noise(bool on) { (void)on; }
static void hw_tune(void) {}
static void hw_gain(void) {}
#endif

/* ------------------------------------------------------------------------------------- the synthetic Kraken */

#define SYN_CHUNK   8192
#define SYN_TAB     65536

static float          syn_rx[SYN_TAB], syn_src[SYN_TAB];   /* receiver noise, the noise source: unit gaussian */
static float complex  syn_cw[4096];             /* the transmitter: 12 counts, a turn in 4096 steps */
static pthread_t      syn_th;
static bool           syn_on;
static int            syn_o[DF_MAXCH];          /* where channel m's sample 0 is in the source's time */
static int            syn_start[DF_MAXCH];      /* when its stream started (what the transfers' times tell) */
static float complex  syn_lo[DF_MAXCH];         /* each tuner's phase and gain: new at every retune */
static _Atomic int    syn_retuned;

static double gauss(unsigned *s) {
    double u = (rand_r(s) + 1.0) / (RAND_MAX + 2.0), v = (rand_r(s) + 1.0) / (RAND_MAX + 2.0);

    return sqrt(-2 * log(u)) * cos(2 * M_PI * v);
}

static void syn_new_lo(unsigned *s) {
    for (int m = 0; m < nch; m++) {
        double ph = 2 * M_PI * (rand_r(s) / (double)RAND_MAX), g = 0.8 + 0.45 * (rand_r(s) / (double)RAND_MAX);

        syn_lo[m] = (float complex)(g * cexp(I * ph));
    }
}

static void *syn_thread(void *arg) {
    (void)arg;
    unsigned       seed = 12345;
    int            span, omin = 0, omax = 0;
    float complex *src;
    double         t_start = now_s();
    int64_t        j = 0;                       /* every channel's next sample */
    uint8_t       *out = malloc(2 * SYN_CHUNK);

    for (int m = 0; m < nch; m++) {
        if (syn_o[m] < omin) omin = syn_o[m];
        if (syn_o[m] > omax) omax = syn_o[m];
    }
    span = omax - omin;
    src = malloc(sizeof(*src) * (size_t)(SYN_CHUNK + span));
    while (atomic_load(&streaming)) {
        double due = t_start + (double)(j + SYN_CHUNK) / rate, t = now_s();

        if (due > t) usleep((useconds_t)((due - t) * 1e6));
        if (atomic_exchange(&syn_retuned, 0)) syn_new_lo(&seed);

        /* the source's time u = j + o_m on channel m; the transmitter: a carrier at the target's offset, on 6 s,
         * off 3 s */
        int64_t u0 = j + omin;
        double  tsec = (double)u0 / rate, bearing = fmod(synth_bearing + synth_rate * tsec + 720, 360);
        bool    on = fmod(tsec, 9.0) < 6.0, noise = atomic_load(&synth_noise);
        double  lambda = DF_C / (fc + f_off);
        double complex sv[DF_MAXCH];

        df_steer_at(&arr, lambda, bearing, sv);
        if (noise) {
            for (int i = 0; i < SYN_CHUNK + span; i++)
                src[i] = 40.0f * (syn_src[(u0 + i) & (SYN_TAB - 1)] + I * syn_src[(u0 + i + 7919) & (SYN_TAB - 1)]) / 1.4142f;
        } else if (on) {                        /* a carrier: a table and a phase that counts 2^32 a turn */
            double   turns = fmod((double)f_off * (double)u0 / rate, 1.0);
            uint32_t ph = (uint32_t)(int64_t)llround((turns < 0 ? turns + 1 : turns) * 4294967296.0);
            uint32_t inc = (uint32_t)(int64_t)llround((double)f_off / rate * 4294967296.0);

            for (int i = 0; i < SYN_CHUNK + span; i++, ph += inc) src[i] = syn_cw[ph >> 20];
        } else
            memset(src, 0, sizeof(*src) * (size_t)(SYN_CHUNK + span));
        int r0 = rand_r(&seed) & (SYN_TAB - 1);

        for (int m = 0; m < nch; m++) {
            const float complex *s = src + (syn_o[m] - omin);
            float complex        a = noise ? syn_lo[m] : syn_lo[m] * (float complex)sv[m] * (m == synth_dead ? 0.03f : 1.0f);
            int                  r = (r0 + 4099 * (m + 1)) & (SYN_TAB - 1);

            for (int i = 0; i < SYN_CHUNK; i++) {
                float complex x = s[i] * a;
                float         re = crealf(x) + 4.0f * syn_rx[(r + 2 * i) & (SYN_TAB - 1)] + 127.5f;
                float         im = cimagf(x) + 4.0f * syn_rx[(r + 2 * i + 1) & (SYN_TAB - 1)] + 127.5f;

                out[2 * i] = (uint8_t)(re < 0 ? 0 : re > 255 ? 255 : re);
                out[2 * i + 1] = (uint8_t)(im < 0 ? 0 : im > 255 ? 255 : im);
            }
            put(&ch[m], out, 2 * SYN_CHUNK);
        }
        for (int m = 0; m < nch; m++) atomic_store(&ch[m].t0, t_start + (double)syn_start[m] / rate);
        j += SYN_CHUNK;
    }
    free(src);
    free(out);
    return NULL;
}

static int syn_open(void) {
    unsigned s = 777;
    static const int o[DF_MAXCH] = {0, -1234, 517, 2890, -77, 33, 1500, -900};
    static const int hw[DF_MAXCH] = {0, 3, -2, 7, 1, 0, 2, -1};    /* the cables' and filters' own delays */

    nch = n_open;
    for (int i = 0; i < SYN_TAB; i++) syn_rx[i] = (float)gauss(&s), syn_src[i] = (float)gauss(&s);
    for (int i = 0; i < 4096; i++) syn_cw[i] = (float complex)(12.0 * cexp(I * 2 * M_PI * i / 4096));
    for (int m = 0; m < nch; m++) {
        snprintf(ch[m].serial, sizeof(ch[m].serial), "SYN%d", m);
        syn_o[m] = o[m] - hw[m];
        syn_start[m] = o[m];
        atomic_store(&ch[m].wr, 0);
        atomic_store(&ch[m].ended, false);
    }
    syn_new_lo(&s);
    atomic_store(&streaming, true);
    syn_on = pthread_create(&syn_th, NULL, syn_thread, NULL) == 0;
    logf_("synthetic Kraken: transmitter at %.0f degrees, turning %.1f a second, dead channel %d", synth_bearing,
          synth_rate, synth_dead);
    return 0;
}

static void syn_close(void) {
    atomic_store(&streaming, false);
    if (syn_on) pthread_join(syn_th, NULL), syn_on = false;
}

/* ------------------------------------------------------------------------------------------- the device, either */

static bool dev_there(void) { return synth || kraken_there(); }
static bool dev_is_open;                       /* apply() leaves the sticks and the engine alone while not */
static int  dev_open(void) {
    int r = synth ? syn_open() : hw_open();

    dev_is_open = r == 0;
    return r;
}
static void dev_close(void) {
    synth ? syn_close() : hw_close();
    dev_is_open = false;
}
static void dev_noise(bool on) { synth ? atomic_store(&synth_noise, on) : hw_noise(on); }
static void dev_tune(void) { synth ? atomic_store(&syn_retuned, 1) : hw_tune(); }
static void dev_gain(void) { synth ? atomic_store(&syn_retuned, 1) : hw_gain(); }

/* ------------------------------------------------------------------------------------------------- the socket */

static int             ls = -1, cl = -1;
static pthread_mutex_t cmd_lock = PTHREAD_MUTEX_INITIALIZER;
static char            cmds[32][160];
static int             ncmds;
static double          cmd_t;                   /* the last command's time (cmd_lock) */
static bool            had_client;              /* (cmd_lock) */
/* Started by page I (not -F): it ends when page I lets go, also while it still waits for the Kraken, and on its own
 * when no client has been there for NO_CLIENT_S (the GUI gone without a Q: a crash, a restart off page I). Page I
 * connects again within a second or two while it is open. Before 2026-11-17 a Q sent while it waited for the sticks
 * was queued and never read: the daemon outlived page I and took the whole Kraken when it came back on USB */
static bool            on_its_own;
#define NO_CLIENT_S 15

static void send_msg(const void *m, size_t n) {
    if (cl >= 0 && send(cl, m, n, MSG_DONTWAIT | MSG_NOSIGNAL) < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        close(cl);
        cl = -1;
    }
}

static void *sock_thread(void *arg) {
    double lost_t = now_s();

    (void)arg;
    while (!atomic_load(&quit_now)) {
        struct pollfd p[2] = {{.fd = ls, .events = POLLIN}, {.fd = cl, .events = POLLIN}};

        if (!on_its_own && cl < 0 && had_client && now_s() - lost_t > NO_CLIENT_S) {
            logf_("no client for %d s: ending", NO_CLIENT_S);
            atomic_store(&quit_now, true);
            break;
        }
        if (poll(p, cl >= 0 ? 2 : 1, 500) <= 0) continue;
        if (p[0].revents & POLLIN) {
            int c = accept(ls, NULL, NULL);

            if (c >= 0) {                       /* one client: a new one replaces the old */
                if (cl >= 0) close(cl);
                cl = c;
                pthread_mutex_lock(&cmd_lock);
                had_client = true;
                cmd_t = now_s();
                pthread_mutex_unlock(&cmd_lock);
                logf_("a client");
            }
        }
        if (cl >= 0 && (p[1].revents & (POLLIN | POLLHUP | POLLERR))) {
            char b[160];
            ssize_t n = recv(cl, b, sizeof(b) - 1, 0);

            if (n <= 0) {
                close(cl);
                cl = -1;
                lost_t = now_s();
                continue;
            }
            b[n] = 0;
            if (b[0] == 'Q') {                  /* at once: the worker may be waiting for the sticks */
                atomic_store(&quit_now, true);
                break;
            }
            pthread_mutex_lock(&cmd_lock);
            if (ncmds < 32) snprintf(cmds[ncmds++], sizeof(cmds[0]), "%s", b);
            cmd_t = now_s();
            pthread_mutex_unlock(&cmd_lock);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------------------ the IQ server (Heimdall's) */

/* The Kraken's samples for other programs (ROADMAP.md, 9): KrakenRF's DoA and passive-radar software take them from
 * Heimdall's IQ server, and this one speaks its protocol. TCP 5000: a client sends "streaming", reads a frame (the
 * 1024-byte header below, then every channel's samples as complex float 32, channel after channel), and asks for each
 * next one with "IQDownload" ("q" or anything else ends it). TCP 5001, the control: 128-byte messages "INIT", "FREQ"
 * (a uint64 Hz), "GAIN" (a uint32 tenths of a dB per channel), "AGC ", "EXIT", each answered "FNSD".
 *
 * What it sends: not the raw 1.024 MS/s (five channels in float are 41 MB/s, far over the radio's WiFi) but the band
 * around the DF target, I BW wide (default 25 kHz, so 64 kS/s and 2.6 MB/s), through a channel filter of its own,
 * each channel lined up (the calibration's whole samples) and corrected (its phase, fraction of a sample and gain at
 * the band's centre). The header's flags say how far to trust it: delay_sync and iq_sync once calibrated, and a frame
 * the noise source was on for is a CAL frame, as Heimdall marks them. One client of each at a time. FREQ puts the
 * Kraken a quarter of its rate under the frequency asked and the DF target on it, as page I's bands do; GAIN sets
 * every channel to the first one's. The filtering runs on its own thread, only while a client is there. Receive
 * only. */

#define IQ_PORT     5000
#define CTL_PORT    5001
#define IQ_SYNC     0x2bf7b95au

typedef struct {                /* Heimdall's iq_header_struct, the same fields in the same order: natural alignment
                                 * (a uint64 on 8 bytes) on the radio's ARM EABI as on a computer's x86-64 */
    uint32_t sync_word, frame_type;
    char     hardware_id[16];
    uint32_t unit_id, active_ant_chs, ioo_type;
    uint64_t rf_center_freq, adc_sampling_freq, sampling_freq;
    uint32_t cpi_length;
    uint64_t time_stamp;
    uint32_t daq_block_index, cpi_index;
    uint64_t ext_integration_cntr;
    uint32_t data_type, sample_bit_depth, adc_overdrive_flags, if_gains[32], delay_sync_flag, iq_sync_flag,
             sync_state, noise_source_state, reserved[192], header_version;
} iq_hdr_t;
_Static_assert(sizeof(iq_hdr_t) == 1024, "Heimdall's IQ header is 1024 bytes");

enum { IQ_DATA = 0, IQ_DUMMY = 1, IQ_CAL = 3 };

static int             iq_bw_set = 25000;           /* the band served, Hz (-I BW; 0 no server) */
static atomic_int      iq_clients;                  /* a client on 5000 */
static atomic_bool     iq_tuned;                    /* the Kraken where an IQ program's FREQ put it (till the GUI tunes) */
static atomic_uint     cal_gen;                     /* kb or the calibration changed (cal_lock) */
static atomic_llong    noise_from = -1, noise_to = -1;  /* CH0's counts the last calibration's noise covered */
static pthread_mutex_t cal_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t iq_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  iq_cond = PTHREAD_COND_INITIALIZER;
static float complex  *iq_frame;                    /* the newest frame, nch x cpi (iq_lock) */
static iq_hdr_t        iq_h;                        /* its header (iq_lock) */
static uint32_t        iq_seq;                      /* frames made (iq_lock) */
static size_t          iq_bytes;                    /* its payload's size (iq_lock) */
static bool            iq_worker_on;                /* (iq_lock) */

/* a command as the GUI's are taken (the engine applies it between blocks) */
static void queue_cmd(const char *c) {
    pthread_mutex_lock(&cmd_lock);
    if (ncmds < 32) snprintf(cmds[ncmds++], sizeof(cmds[0]), "%s", c);
    pthread_mutex_unlock(&cmd_lock);
}
static bool read_at(int64_t j, int n, uint8_t *const dst[]);
static bool wait_for(int64_t j, int n, double timeout);
static int64_t wr_of(int m);

/* The filtering: blocks from the rings as the engine reads them (a few blocks behind the newest), the band cut out
 * and decimated, corrected, gathered into frames of a quarter of a second */
static void *iq_worker(void *arg) {
    df_chan_t      c;
    bool           c_on = false;
    uint8_t       *raw[DF_MAXCH] = {0};
    float complex *dec[DF_MAXCH] = {0}, *acc = NULL, corr[DF_MAXCH];
    int            n = 0, cpi = 0, pos = 0, ch_n = 0;
    int64_t        j = -1, total = 0;
    uint32_t       gen = ~0u, c_fc = 0, c_rate = 0, c_bw = 0, made = 0;
    int32_t        c_off = 0;
    bool           noisy = false, cal_ok = false;

    (void)arg;
    while (!atomic_load(&quit_now) && atomic_load(&iq_clients) > 0) {
        int k = nch;

        if (!k || !rate || !atomic_load(&streaming)) {
            usleep(100000);
            continue;
        }
        /* a new rate, centre, target or band: the filter and the frame begin again */
        if (!c_on || c_fc != fc || c_rate != rate || c_off != f_off || c_bw != (uint32_t)iq_bw_set || ch_n != k) {
            if (c_on) df_chan_free(&c);
            n = (int)(rate / 10);
            c_on = df_chan_init(&c, k, rate, f_off, iq_bw_set, n) == 0;
            if (!c_on) {
                usleep(200000);
                continue;
            }
            for (int m = 0; m < DF_MAXCH; m++) {
                free(raw[m]), free(dec[m]);
                raw[m] = m < k ? malloc(2 * (size_t)n) : NULL;
                dec[m] = m < k ? malloc(sizeof(float complex) * (size_t)n) : NULL;
            }
            for (cpi = 1024; cpi * 2 <= c.fs_out * 0.25; cpi *= 2) ;     /* a quarter of a second, a power of two */
            free(acc);
            acc = malloc(sizeof(float complex) * (size_t)cpi * (size_t)k);
            c_fc = fc, c_rate = rate, c_off = f_off, c_bw = (uint32_t)iq_bw_set, ch_n = k;
            j = -1, pos = 0, noisy = false;
            if (!acc) break;
        }
        /* the calibration as it is now: its corrections at the band's centre (cal_lock: the engine changes it) */
        if (gen != atomic_load(&cal_gen) || j < 0) {
            pthread_mutex_lock(&cal_lock);
            gen = atomic_load(&cal_gen);
            cal_ok = have_cal;
            for (int m = 0; m < k; m++) corr[m] = cal_ok ? df_cal_corr(&cal, m, f_off, rate) : 1;
            j = wr_of(0) - 3 * n;                           /* a few blocks behind the newest, lined up again */
            pthread_mutex_unlock(&cal_lock);
            pos = 0, noisy = false;
        }
        if (!wait_for(j, n, 0.5)) continue;
        pthread_mutex_lock(&cal_lock);
        bool got = gen == atomic_load(&cal_gen) && read_at(j, n, raw);
        pthread_mutex_unlock(&cal_lock);
        if (!got) {
            j = -1;
            continue;
        }
        int64_t nf = atomic_load(&noise_from), nt = atomic_load(&noise_to);
        if (nf >= 0 && j < (nt >= 0 ? nt : INT64_MAX) && j + n > nf) noisy = true;
        j += n;
        int got_n = df_chan_run(&c, (const uint8_t *const *)raw, n, dec);
        for (int i = 0; i < got_n; i++) {
            for (int m = 0; m < k; m++) acc[(size_t)m * (size_t)cpi + (size_t)pos] = dec[m][i] * corr[m];
            if (++pos < cpi) continue;
            /* a frame: published for the server, the newest only */
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            pthread_mutex_lock(&iq_lock);
            size_t need = sizeof(float complex) * (size_t)cpi * (size_t)k;
            if (iq_bytes != need) {
                free(iq_frame);
                iq_frame = malloc(need);
                iq_bytes = iq_frame ? need : 0;
            }
            if (iq_frame) memcpy(iq_frame, acc, need);
            memset(&iq_h, 0, sizeof(iq_h));
            iq_h.sync_word = IQ_SYNC;
            iq_h.frame_type = noisy ? IQ_CAL : cal_ok ? IQ_DATA : IQ_DUMMY;
            snprintf(iq_h.hardware_id, sizeof(iq_h.hardware_id), "k5");
            iq_h.active_ant_chs = (uint32_t)k;
            iq_h.rf_center_freq = (uint64_t)((int64_t)fc + f_off);
            iq_h.adc_sampling_freq = rate;
            iq_h.sampling_freq = (uint64_t)llround(c.fs_out);
            iq_h.cpi_length = (uint32_t)cpi;
            iq_h.time_stamp = (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
            iq_h.daq_block_index = iq_h.cpi_index = made++;
            total += cpi;
            iq_h.ext_integration_cntr = (uint64_t)total;
            iq_h.data_type = 3;                             /* decimated IQ, as Heimdall's decimator says it */
            iq_h.sample_bit_depth = 32;                     /* complex float 32 */
            for (int m = 0; m < k; m++) iq_h.if_gains[m] = (uint32_t)gain;
            iq_h.delay_sync_flag = iq_h.iq_sync_flag = cal_ok;
            iq_h.sync_state = cal_ok ? 6 : 0;
            iq_h.noise_source_state = noisy;
            iq_h.header_version = 7;
            iq_seq++;
            pthread_cond_broadcast(&iq_cond);
            pthread_mutex_unlock(&iq_lock);
            pos = 0, noisy = false;
        }
    }
    if (c_on) df_chan_free(&c);
    for (int m = 0; m < DF_MAXCH; m++) free(raw[m]), free(dec[m]);
    free(acc);
    pthread_mutex_lock(&iq_lock);
    iq_worker_on = false;
    pthread_mutex_unlock(&iq_lock);
    return NULL;
}

static int listen_tcp(int port) {
    int                fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0), one = 1;
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons((uint16_t)port), .sin_addr.s_addr = INADDR_ANY};

    if (fd < 0) return -1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) || listen(fd, 1)) {
        logf_("IQ server: port %d not open (%s)", port, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

static void set_timeouts(int fd, int sec) {
    struct timeval tv = {sec, 0};

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static bool send_all(int fd, const void *b, size_t n) {
    const uint8_t *p = b;

    while (n) {
        ssize_t r = send(fd, p, n, MSG_NOSIGNAL);
        if (r <= 0) return false;
        p += r, n -= (size_t)r;
    }
    return true;
}

/* One client at a time on 5000: frames as it asks for them, the newest each time (a slow client skips some) */
static void *iq_thread(void *arg) {
    int lfd = *(int *)arg;

    while (!atomic_load(&quit_now)) {
        struct pollfd p = {.fd = lfd, .events = POLLIN};
        if (poll(&p, 1, 500) <= 0) continue;
        int fd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
        if (fd < 0) continue;
        char     b[64];
        uint32_t last = 0;
        bool     first = true;

        set_timeouts(fd, 10);
        ssize_t r = recv(fd, b, sizeof(b) - 1, 0);             /* "streaming" */
        if (r <= 0) {
            close(fd);
            continue;
        }
        atomic_store(&iq_clients, 1);
        pthread_mutex_lock(&iq_lock);
        last = iq_seq;
        if (!iq_worker_on) {
            pthread_t t;
            iq_worker_on = pthread_create(&t, NULL, iq_worker, NULL) == 0;
            if (iq_worker_on) pthread_detach(t);
        }
        pthread_mutex_unlock(&iq_lock);
        logf_("IQ server: a client");
        while (!atomic_load(&quit_now)) {
            if (!first) {                                   /* the next frame asked for */
                r = recv(fd, b, sizeof(b) - 1, 0);
                if (r <= 0) break;
                b[r] = 0;
                if (strncmp(b, "IQDownload", 10)) break;
            }
            first = false;
            /* the next new frame, then sent outside the lock (a copy: the worker goes on meanwhile) */
            pthread_mutex_lock(&iq_lock);
            struct timespec until;
            clock_gettime(CLOCK_REALTIME, &until);
            until.tv_sec += 3;
            while (iq_seq == last && !atomic_load(&quit_now))
                if (pthread_cond_timedwait(&iq_cond, &iq_lock, &until)) break;
            bool     have = iq_seq != last && iq_frame;
            iq_hdr_t h = iq_h;
            size_t   nb = iq_bytes;
            void    *copy = have ? malloc(nb) : NULL;
            if (copy) memcpy(copy, iq_frame, nb);
            last = iq_seq;
            pthread_mutex_unlock(&iq_lock);
            if (!copy) {                                    /* nothing yet (opening, no streams): a dummy frame */
                memset(&h, 0, sizeof(h));
                h.sync_word = IQ_SYNC, h.frame_type = IQ_DUMMY, h.header_version = 7, h.sample_bit_depth = 32;
                snprintf(h.hardware_id, sizeof(h.hardware_id), "k5");
                if (!send_all(fd, &h, sizeof(h))) break;
                continue;
            }
            bool ok = send_all(fd, &h, sizeof(h)) && send_all(fd, copy, nb);
            free(copy);
            if (!ok) break;
        }
        close(fd);
        atomic_store(&iq_clients, 0);
        logf_("IQ server: the client went");
    }
    return NULL;
}

/* The control on 5001: what KrakenRF's software sends Heimdall's hardware controller */
static void *ctl_thread(void *arg) {
    int lfd = *(int *)arg;

    while (!atomic_load(&quit_now)) {
        struct pollfd p = {.fd = lfd, .events = POLLIN};
        if (poll(&p, 1, 500) <= 0) continue;
        int fd = accept4(lfd, NULL, NULL, SOCK_CLOEXEC);
        if (fd < 0) continue;
        set_timeouts(fd, 30);
        while (!atomic_load(&quit_now)) {
            uint8_t m[128];
            ssize_t r = recv(fd, m, sizeof(m), MSG_WAITALL);
            char    reply[128] = {0}, c[64];

            if (r < 4) break;
            if (!memcmp(m, "EXIT", 4)) break;
            memcpy(reply, "FNSD", 4);
            if (!memcmp(m, "FREQ", 4) && r >= 12) {
                uint64_t hz;
                memcpy(&hz, m + 4, sizeof(hz));
                if (hz >= 24000000ull && hz <= 1766000000ull) {
                    snprintf(c, sizeof(c), "F %llu", (unsigned long long)hz);
                    queue_cmd(c);
                    logf_("IQ control: FREQ %.4f MHz", hz / 1e6);
                } else {
                    memcpy(reply, "FAIL", 4);
                }
            } else if (!memcmp(m, "GAIN", 4) && r >= 8) {
                uint32_t g;
                memcpy(&g, m + 4, sizeof(g));
                snprintf(c, sizeof(c), "g %u", g);
                queue_cmd(c);
            } else if (memcmp(m, "INIT", 4) && memcmp(m, "AGC ", 4)) {
                memcpy(reply, "FAIL", 4);
            }
            if (!send_all(fd, reply, sizeof(reply))) break;
        }
        close(fd);
    }
    return NULL;
}

static void iq_start(void) {
    static int iq_fd = -1, ctl_fd = -1;
    pthread_t  t;

    if (iq_bw_set <= 0) return;
    if ((iq_fd = listen_tcp(IQ_PORT)) >= 0 && pthread_create(&t, NULL, iq_thread, &iq_fd) == 0) pthread_detach(t);
    if ((ctl_fd = listen_tcp(CTL_PORT)) >= 0 && pthread_create(&t, NULL, ctl_thread, &ctl_fd) == 0) pthread_detach(t);
    if (iq_fd >= 0) logf_("IQ server: Heimdall's protocol on %d (control %d), %d Hz around the target", IQ_PORT, CTL_PORT,
                          iq_bw_set);
}

/* ------------------------------------------------------------------------------------------------- the worker */

static df_engine_t eng;
static atomic_uint set_gen;            /* the engine's settings changed (apply, engine_setup): other targets */
static atomic_llong J_pub = -1;        /* where the block the main engine last read starts (CH0's count) */
static int         nsrc_set = 1;       /* k: kept across the engine's rebuilds (a new rate), as the method is */
static bool        fb_on;
static int         smooth_set;         /* o: spatial smoothing's subarrays (0 off) */
static int         blk_n;
static uint8_t    *blk[DF_MAXCH];
static int64_t     J;                           /* CH0's next block */
static bool        eng_on;

static int64_t wr_of(int m) { return atomic_load(&ch[m].wr); }

/* n samples of channel m from its own count a, out of the ring */
static void copy_ch(int m, int64_t a, int n, uint8_t *dst) {
    size_t at = (size_t)(a & (RING - 1)), first = RING - at < (size_t)n ? RING - at : (size_t)n;

    memcpy(dst, ch[m].ring + 2 * at, 2 * first);
    if (first < (size_t)n) memcpy(dst + 2 * first, ch[m].ring, 2 * ((size_t)n - first));
}

/* the channels' bytes from j (CH0's count) on, each kb[m] later, into dst; false if not written yet or gone */
static bool read_at(int64_t j, int n, uint8_t *const dst[]) {
    for (int m = 0; m < nch; m++) {
        int64_t a = j + kb[m];

        if (a < 0 || a + n > wr_of(m) || a < wr_of(m) - RING) return false;
    }
    for (int m = 0; m < nch; m++) copy_ch(m, j + kb[m], n, dst[m]);
    return true;
}

/* wait till channel data from j for n samples is there (and none of it gone); false: timed out or quitting */
static bool wait_for(int64_t j, int n, double timeout) {
    double until = now_s() + timeout;

    while (!atomic_load(&quit_now) && now_s() < until) {
        bool ok = true;

        for (int m = 0; m < nch && ok; m++) ok = j + kb[m] + n <= wr_of(m);
        if (ok) return true;
        for (int m = 0; m < nch; m++)
            if (atomic_load(&ch[m].ended)) return false;
        usleep(4000);
    }
    return false;
}

static void engine_setup(void) {
    if (eng_on) df_engine_free(&eng);
    memset(&eng, 0, sizeof(eng));
    eng.nch = arr.n <= nch ? arr.n : nch;
    eng.fs = rate;
    eng.fc = fc;
    eng.f = f_off;
    eng.bw = bw;
    eng.arr = arr;
    if (eng.arr.n > eng.nch) eng.arr.n = eng.nch;
    eng.method = (df_method_t)method;
    eng.nsrc = nsrc_set;
    eng.fb = fb_on;
    eng.smooth = smooth_set;
    eng.sql_db = sql / 10.0;
    eng.cal_every_s = cal_every > 0 ? cal_every : -1;
    eng.user_off = user_off;
    blk_n = (int)(rate / 10);
    eng_on = df_engine_init(&eng, blk_n) == 0;
    if (!eng_on) logf_("engine: could not start (%d channels, array of %d)", eng.nch, arr.n);
    if (have_cal) df_engine_set_cal(&eng, &cal);
    atomic_fetch_add(&set_gen, 1);
    atomic_store(&J_pub, -1);
    for (int m = 0; m < DF_MAXCH; m++) {
        free(blk[m]);
        blk[m] = m < nch ? malloc(2 * (size_t)blk_n) : NULL;
    }
}

static int16_t ddeg(double d) { return d < 0 ? -1 : (int16_t)lrint(fmod(d, 360) * 10) % 3600; }
static int16_t clamp16(double v) { return (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : lrint(v)); }

/* ------------------------------------------------------------------------------------ other targets (ROADMAP 8)
 * Up to DF_PROTO_TARGETS more targets in the Kraken's passband besides the main one (commands 1-3): each its own
 * engine (channel filter, squelch, bearing) on a thread of its own, reading the same rings the block the main engine
 * last read, with the main one's settings and calibration. The main target never waits for them: one that cannot
 * keep up takes the newest block and says how many it kept up with (done_pct). Their threads go on CPU2 (with the
 * readers, which come first), CPU0 (the GUI's) and CPU1 (the USB interrupt's), at nice 10 so the GUI and the rest
 * come first: the engine's own core has no room for another */

typedef struct {
    int32_t     off;                    /* set: the offset from the centre (tg_lock) */
    uint32_t    bw;                     /* 0: off (tg_lock) */
    bool        alive;                  /* its thread runs (tg_lock) */
    bool        joinable;
    pthread_t   th;
    df_tg_t     out;                    /* the newest, for DFTG (tg_lock) */
    double      last_good_at, last_bearing, last_mirror, last_uncert;
    double      held_bearing, held_mirror, held_uncert, held_fit;
} target_t;

static target_t        tg[DF_PROTO_TARGETS];
static df_engine_t     tg_eng[DF_PROTO_TARGETS];
static pthread_mutex_t tg_lock = PTHREAD_MUTEX_INITIALIZER;
static const int       tg_cpu[DF_PROTO_TARGETS] = {2, 0, 1};    /* measured with five sticks streaming: CPU2 81 % with
                                                                 * one target and the readers, CPU1 full of the USB
                                                                 * interrupt's copying (a target there kept up with
                                                                 * 74 % of blocks), CPU0 the GUI's at about 25 % */
#define TG_NICE 10                                              /* below the GUI and everything else */

/* the newest estimate kept as DFDO keeps the main one's: a held bearing for HOLD_S once the gate shuts */
static void tg_take(target_t *t, const df_est_t *e, int done_pct) {
    double  now = now_s();
    df_tg_t o;

    memset(&o, 0, sizeof(o));
    o.on = 1;
    o.gate = e->gate;
    o.done_pct = (uint8_t)(done_pct > 100 ? 100 : done_pct);
    o.fit_pct = (uint8_t)lrint(fmin(1, fmax(0, e->doa.fit)) * 100);
    if (e->gate) {
        if (t->last_good_at < 0 || now - t->last_good_at >= HOLD_S) t->held_bearing = -1;
        t->last_good_at = now;
        t->last_bearing = e->doa.bearing;
        t->last_mirror = e->doa.mirror;
        t->last_uncert = e->uncert_deg;
        if (e->doa.fit >= HOLD_FIT) {
            t->held_bearing = e->doa.bearing;
            t->held_mirror = e->doa.mirror;
            t->held_uncert = e->uncert_deg;
            t->held_fit = e->doa.fit;
        }
    }
    if (!e->gate && t->held_bearing >= 0 && now - t->last_good_at < HOLD_S) {
        o.bearing_ddeg = ddeg(t->held_bearing);
        o.mirror_ddeg = ddeg(t->held_mirror);
        o.uncert_ddeg = clamp16(t->held_uncert * 10);
        o.fit_pct = (uint8_t)lrint(fmin(1, fmax(0, t->held_fit)) * 100);
    } else {
        o.bearing_ddeg = ddeg(t->last_bearing);
        o.mirror_ddeg = ddeg(t->last_mirror);
        o.uncert_ddeg = clamp16(t->last_uncert * 10);
    }
    o.level_ddb = clamp16(e->level_db * 10);
    o.age_ds = t->last_good_at < 0 ? 0xffff : (uint16_t)fmin(65534, (now - t->last_good_at) * 10);
    pthread_mutex_lock(&tg_lock);
    o.off_hz = t->off;
    o.bw_hz = t->bw;
    t->out = o;
    pthread_mutex_unlock(&tg_lock);
}

static void *tg_worker(void *arg) {
    int          i = (int)(intptr_t)arg;
    target_t    *t = &tg[i];
    df_engine_t *e = &tg_eng[i];
    bool         e_on = false;
    uint8_t     *raw[DF_MAXCH] = {0};
    int          n = 0, done = 0;
    int64_t      j = -1;
    unsigned     sgen = ~0u, cgen = ~0u;
    int32_t      e_off = 0;
    uint32_t     e_bw = 0, e_fc = 0, e_rate = 0;
    double       win_t = now_s();
    int          done_pct = 100;

    pin_self(tg_cpu[i], 0, "a target");
    if (setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), TG_NICE)) logf_("target %d: nice not set", i + 2);
    t->last_good_at = -1, t->held_bearing = -1, t->last_bearing = t->last_mirror = -1;
    for (;;) {
        pthread_mutex_lock(&tg_lock);
        int32_t  off = t->off;
        uint32_t bw = t->bw;

        if (!bw || atomic_load(&quit_now)) {            /* cleared: the thread ends (said under the lock, so a new
                                                         * setting finds it gone and starts another) */
            t->alive = false;
            memset(&t->out, 0, sizeof(t->out));
            pthread_mutex_unlock(&tg_lock);
            break;
        }
        pthread_mutex_unlock(&tg_lock);
        if (!eng_on || !atomic_load(&streaming) || !nch || !rate) {
            usleep(100000);
            continue;
        }
        /* the settings, the band or this target changed: its engine anew, from the main one's settings */
        if (!e_on || sgen != atomic_load(&set_gen) || off != e_off || bw != e_bw || fc != e_fc || rate != e_rate) {
            if (e_on) df_engine_free(e);
            sgen = atomic_load(&set_gen);
            memset(e, 0, sizeof(*e));
            e->nch = arr.n <= nch ? arr.n : nch;
            e->fs = rate;
            e->fc = fc;
            e->f = off;
            e->bw = bw;
            e->arr = arr;
            if (e->arr.n > e->nch) e->arr.n = e->nch;
            e->method = (df_method_t)method;
            e->nsrc = nsrc_set;
            e->fb = fb_on;
            e->smooth = smooth_set;
            e->sql_db = sql / 10.0;
            e->cal_every_s = -1;                        /* the main engine calibrates */
            e->user_off = user_off;
            n = (int)(rate / 10);
            e_on = df_engine_init(e, n) == 0;
            for (int m = 0; m < DF_MAXCH; m++) {
                free(raw[m]);
                raw[m] = m < nch ? malloc(2 * (size_t)n) : NULL;
            }
            e_off = off, e_bw = bw, e_fc = fc, e_rate = rate;
            cgen = ~0u, j = -1;
            t->last_good_at = -1, t->held_bearing = -1, t->last_bearing = t->last_mirror = -1;
            if (!e_on) {
                usleep(200000);
                continue;
            }
        }
        if (cgen != atomic_load(&cal_gen)) {
            pthread_mutex_lock(&cal_lock);
            cgen = atomic_load(&cal_gen);
            if (have_cal) df_engine_set_cal(e, &cal);
            pthread_mutex_unlock(&cal_lock);
            j = -1;
        }
        /* the blocks the main engine has read, in order (it reads in bursts after a wait); more than four behind,
         * skip to its newest */
        int64_t newest = atomic_load(&J_pub);

        if (newest < 0 || (j >= 0 && j > newest)) {
            usleep(10000);
            continue;
        }
        if (j < 0 || newest - j > 4 * (int64_t)n) j = newest;
        pthread_mutex_lock(&cal_lock);
        bool got = cgen == atomic_load(&cal_gen) && read_at(j, n, raw);
        pthread_mutex_unlock(&cal_lock);
        if (!got) {
            j = -1;
            usleep(10000);
            continue;
        }
        j += n;
        df_est_t est;

        df_engine_block(e, (const uint8_t *const *)raw, n, &est);
        done++;
        double now = now_s();

        if (now - win_t >= 1) {
            done_pct = (int)lrint(100.0 * done / ((now - win_t) * rate / n));
            done = 0;
            win_t = now;
        }
        tg_take(t, &est, done_pct);
    }
    if (e_on) df_engine_free(e);
    for (int m = 0; m < DF_MAXCH; m++) free(raw[m]);
    return NULL;
}

/* "1|2|3 OFFSET_HZ BW_HZ": a target set, moved or cleared; its thread started when it is not running */
static void tg_set(int i, int32_t off, uint32_t bw) {
    target_t *t = &tg[i];

    pthread_mutex_lock(&tg_lock);
    t->off = off;
    t->bw = bw;
    bool need = bw && !t->alive;

    if (need) t->alive = true;
    pthread_mutex_unlock(&tg_lock);
    if (!need) return;
    if (t->joinable) pthread_join(t->th, NULL), t->joinable = false;     /* the one before, ended */
    t->joinable = pthread_create(&t->th, NULL, tg_worker, (void *)(intptr_t)i) == 0;
    if (!t->joinable) {
        pthread_mutex_lock(&tg_lock);
        t->alive = false;
        pthread_mutex_unlock(&tg_lock);
        logf_("target %d: no thread", i + 1);
    }
}

static void tg_end_all(void) {
    for (int i = 0; i < DF_PROTO_TARGETS; i++) {
        pthread_mutex_lock(&tg_lock);
        tg[i].bw = 0;
        pthread_mutex_unlock(&tg_lock);
        if (tg[i].joinable) pthread_join(tg[i].th, NULL), tg[i].joinable = false;
    }
}

static void send_targets(uint32_t seq) {
    df_tg_msg_t m;
    bool        any = false;

    memset(&m, 0, sizeof(m));
    memcpy(m.tag, "DFTG", 4);
    m.seq = seq;
    pthread_mutex_lock(&tg_lock);
    for (int i = 0; i < DF_PROTO_TARGETS; i++) {
        if (tg[i].bw) {
            m.tg[i] = tg[i].out;
            m.tg[i].on = 1;
            m.tg[i].off_hz = tg[i].off;
            m.tg[i].bw_hz = tg[i].bw;
            if (!tg[i].out.on) m.tg[i].bearing_ddeg = m.tg[i].mirror_ddeg = -1, m.tg[i].age_ds = 0xffff;
            any = true;
        }
    }
    pthread_mutex_unlock(&tg_lock);
    if (any) send_msg(&m, sizeof(m));
}

static void send_state(void);

/* the whole-sample delays from the transfers' times, before any noise: within a few ms */
static void coarse_align(void) {
    double t00 = atomic_load(&ch[0].t0);

    pthread_mutex_lock(&cal_lock);
    for (int m = 0; m < nch; m++) kb[m] = llround((t00 - atomic_load(&ch[m].t0)) * rate);
    atomic_fetch_add(&cal_gen, 1);
    pthread_mutex_unlock(&cal_lock);
}

static float pw_mean(const float complex *x, int n) {
    double s = 0;

    for (int i = 0; i < n; i++) s += crealf(x[i]) * crealf(x[i]) + cimagf(x[i]) * cimagf(x[i]);
    return (float)(s / n);
}

/* One noise block: the noise source on, DF_CAL_N samples a channel after it settles, off again. search: look for
 * the whole delays (the first time, after a slip); else they are known and only checked */
static bool calibrate(bool search) {
    static float complex *x[DF_MAXCH];
    float                 pre[DF_MAXCH];
    int                   n = DF_CAL_N;
    df_cal_t              c;
    int64_t               j;
    bool                  ok = false;

    for (int m = 0; m < nch; m++)
        if (!x[m] && !(x[m] = malloc(sizeof(float complex) * (size_t)n))) return false;
    st = DF_ST_CAL;
    send_state();

    /* each channel's level before, from its newest 16k samples */
    for (int m = 0; m < nch; m++) {
        uint8_t b[2 * 16384];

        copy_ch(m, wr_of(m) - 16384, 16384, b);
        df_u8_to_cf(b, 16384, x[m]);
        pre[m] = pw_mean(x[m], 16384);
    }

    atomic_store(&noise_to, -1);
    atomic_store(&noise_from, wr_of(0));            /* the IQ server marks frames over it as CAL */
    dev_noise(true);
    j = wr_of(0) + (int64_t)(SETTLE_S * rate);
    if (!wait_for(j, n, 3)) {
        dev_noise(false);
        atomic_store(&noise_to, wr_of(0) + (int64_t)(SETTLE_S * rate));
        goto failed;
    }
    {
        uint8_t *tmp = malloc(2 * (size_t)n);

        for (int m = 0; m < nch && tmp; m++) {
            copy_ch(m, j + kb[m], n, tmp);
            df_u8_to_cf(tmp, n, x[m]);
        }
        free(tmp);
    }
    dev_noise(false);
    J = wr_of(0) + (int64_t)(SETTLE_S * rate);  /* estimates again once the antennas are back */
    atomic_store(&noise_to, J);

    float rises[DF_MAXCH];

    for (int m = 0; m < nch; m++) {
        float rise = rises[m] = 10 * log10f(pw_mean(x[m], n) / (pre[m] + 1e-12f));

        if (rise < 6) {
            snprintf(why, sizeof(why), "CH%d: the noise source rose only %.1f dB", m, rise);
            goto failed;
        }
    }
    {
        static const int zero[DF_MAXCH];
        int              rc = -1;

        if (!search) rc = df_cal_run(x, nch, n, MAXLAG, zero, &c);
        if (rc) rc = df_cal_run(x, nch, n, MAXLAG, NULL, &c);
        if (rc) {
            snprintf(why, sizeof(why), "%.63s", c.why);
            logf_("noise rose %.0f %.0f %.0f %.0f dB", rises[0], nch > 1 ? rises[1] : 0, nch > 2 ? rises[2] : 0,
                  nch > 3 ? rises[3] : 0);
            goto failed;
        }
    }
    pthread_mutex_lock(&cal_lock);
    for (int m = 1; m < nch; m++) kb[m] += c.k[m];
    cal = c;
    have_cal = true;
    atomic_fetch_add(&cal_gen, 1);
    pthread_mutex_unlock(&cal_lock);
    cal_at = now_s();
    cals++;
    why[0] = 0;
    ok = true;
    if (eng_on) df_engine_set_cal(&eng, &cal);
    {
        char l[260];
        int  n = 0;

        n += snprintf(l, sizeof(l), " noise +%.0f dB on CH0;", rises[0]);
        for (int m = 1; m < nch; m++)
            n += snprintf(l + n, sizeof(l) - (size_t)n, " CH%d %lld %+.1f deg %.2f dB (+%.0f, rip %.1f res %.2f)", m,
                          (long long)kb[m], cal.phi[m] * 180 / M_PI, 20 * log10(cal.gain[m]), rises[m],
                          cal.ripple_deg[m], cal.resid_deg[m]);
        logf_("calibrated:%s", l);
    }
    slip_shown = false;
    return true;
failed:
    cal_fails++;
    eng.since_cal = 0;                          /* the timer waits its whole interval again (not a retry a block) */
    if (!why[0]) snprintf(why, sizeof(why), "the noise block did not come in");
    logf_("calibration failed: %s", why);
    J = wr_of(0);
    return ok;
}

/* ------------------------------------------------------------------------------------------------- messages */


static int ch_cond(int m) {
    if (m >= nch || atomic_load(&ch[m].ended)) return DF_CH_GONE;
    if (user_off & (1u << m)) return DF_CH_OFF;
    if (!have_cal) return DF_CH_NOCAL;
    if (m < eng.nch && last.bad_ch == m) return DF_CH_SLIP;
    if (last.clip[m] > 0.01) return DF_CH_CLIP;
    if (m >= eng.nch) return DF_CH_OFF;
    if (eng.low[m]) return DF_CH_LOW;
    if (last.ch_db[m] < eng.sql_db) return DF_CH_QUIET;
    return DF_CH_OK;
}

static void send_state(void) {
    df_st_msg_t s;

    memset(&s, 0, sizeof(s));
    memcpy(s.tag, "DFST", 4);
    s.ver = DF_PROTO_VER;
    s.state = (uint8_t)st;
    s.nch = (uint8_t)nch;
    s.synth = synth;
    s.method = (uint8_t)method;
    s.arr_kind = (uint8_t)arr.kind;
    s.arr_n = (uint8_t)arr.n;
    s.user_off = (uint8_t)user_off;
    s.fc_hz = fc;
    s.f_off_hz = f_off;
    s.bw_hz = bw;
    s.rate = rate;
    s.gain_tenths = (int16_t)gain;
    s.sql_tenths = (int16_t)sql;
    s.cal_every_s = (uint16_t)(cal_every > 0 ? cal_every : 0);
    s.cal_age_s = cal_at < 0 ? 0xffff : (uint16_t)fmin(65534, now_s() - cal_at);
    s.cals = (uint32_t)cals;
    s.cal_fails = (uint32_t)cal_fails;
    s.slips = (uint32_t)slips;
    s.overruns = (uint32_t)overruns;
    s.arr_size_mm = (uint16_t)lrint(arr.size * 1000);
    for (int m = 0; m < arr.n && m < DF_PROTO_MAXCH; m++) {
        s.arr_x_mm[m] = clamp16(arr.x[m] * 1000);
        s.arr_y_mm[m] = clamp16(arr.y[m] * 1000);
    }
    s.cpu_pct = (uint16_t)(cpu_pct < 0 ? 0 : cpu_pct > 65535 ? 65535 : cpu_pct);
    s.nsrc_set = (uint8_t)(eng.nsrc < 0 ? 0 : eng.nsrc ? eng.nsrc : 1);
    s.fb = eng.fb;
    s.smooth = (uint8_t)(eng.smooth > 1 ? eng.smooth : 0);
    s.iq_clients = (uint8_t)((atomic_load(&iq_clients) ? 1 : 0) | (atomic_load(&iq_tuned) ? 2 : 0));
    snprintf(s.why, sizeof(s.why), "%s", why[0] ? why : note);
    for (int m = 0; m < nch && m < DF_PROTO_MAXCH; m++) {
        df_ch_t *c = &s.ch[m];

        snprintf(c->serial, sizeof(c->serial), "%.7s", ch[m].serial);
        c->cond = (uint8_t)ch_cond(m);
        c->used = (last.used >> m) & 1;
        if (have_cal) {
            c->cal_k = clamp16((double)kb[m]);
            c->cal_phase_ddeg = clamp16(remainder(cal.phi[m] * 180 / M_PI, 360) * 10);
            c->cal_gain_mdb = clamp16(20 * log10(cal.gain[m] > 0 ? cal.gain[m] : 1) * 100);
            c->cal_margin_ddb = clamp16(cal.margin_db[m] * 10);
            c->cal_resid_cdeg = clamp16(cal.resid_deg[m] * 100);
            c->noise_ddbfs = clamp16(cal.level_db[m] * 10);
        }
    }
    send_msg(&s, sizeof(s));
}

/* ------------------------------------------------------------------------------------------------ the bearing log */

/* Each bearing over the squelch, two a second at most, in a CSV file a day (UTC) on the DATA partition (ROADMAP.md,
 * 5): when, where (gpsd's fix), the frequency, the bearing from ANT 0, the heading and where it came from, the compass
 * bearing, its +- and fit, the level, the method and the other sources. Off unless asked (l 1); h DEG is the heading
 * set by hand on page I, which the log uses when gpsd has none. Local only: nothing leaves the radio */
#define LOG_DIR     "/mnt/df_log"
#define LOG_EVERY_S 0.5

static bool   log_on;
static int    hand_heading;
static FILE  *log_f;
static char   log_day[16];
static double log_last;

static void log_bearing(const df_est_t *e) {
    static const char *const mnames[] = {"MUSIC", "BARTLETT", "CAPON", "MEM", "TNA", "ROOT-MUSIC"};
    double                   t = now_s(), hd = 0, lat = 0, lon = 0;
    struct timespec          ts;
    struct tm                tm;
    char                     day[16], when[32];

    if (!log_on || !e->gate || t - log_last < LOG_EVERY_S) return;
    log_last = t;
    clock_gettime(CLOCK_REALTIME, &ts);
    gmtime_r(&ts.tv_sec, &tm);
    strftime(day, sizeof(day), "%Y-%m-%d", &tm);
    strftime(when, sizeof(when), "%Y-%m-%dT%H:%M:%S", &tm);
    if (!log_f || strcmp(day, log_day)) {
        char path[64];
        bool fresh;

        if (log_f) fclose(log_f);
        mkdir(LOG_DIR, 0755);
        snprintf(path, sizeof(path), LOG_DIR "/%s.csv", day);
        fresh = access(path, F_OK) != 0;
        if (!(log_f = fopen(path, "a"))) return;
        snprintf(log_day, sizeof(log_day), "%s", day);
        if (fresh)
            fprintf(log_f, "utc,freq_hz,bearing_ant0,heading,heading_from,bearing,plus_minus,fit_pct,level_db,snr_db,"
                           "method,sources,other_bearings,lat,lon\n");
    }
    pthread_mutex_lock(&gps_lock);
    int  src = df_gps_heading(&gps, t, &hd);
    bool fix = df_gps_fix(&gps, t, &lat, &lon);
    pthread_mutex_unlock(&gps_lock);
    const char *from = src == DF_HEAD_COMPASS ? "compass" : src == DF_HEAD_COURSE ? "gps course" : hand_heading ? "set" : "";
    if (!src) hd = hand_heading;
    char other[48] = "";
    for (int i = 0; i < e->doa.npeaks && i < 3; i++)
        snprintf(other + strlen(other), sizeof(other) - strlen(other), "%s%.1f", i ? " " : "", e->doa.peaks[i]);
    fprintf(log_f, "%s.%03ldZ,%lld,%.1f,", when, ts.tv_nsec / 1000000, (long long)fc + f_off, e->doa.bearing);
    if (*from) fprintf(log_f, "%.1f,%s,%.1f,", hd, from, fmod(e->doa.bearing + hd, 360.0));
    else fprintf(log_f, ",,,");
    fprintf(log_f, "%.1f,%.0f,%.1f,%.1f,%s,%d,%s,", e->uncert_deg, e->doa.fit * 100, e->level_db, e->doa.snr_db,
            mnames[method >= 0 && method < 5 ? method : 0], e->doa.nsrc > 0 ? e->doa.nsrc : 1, other);
    if (fix) fprintf(log_f, "%.7f,%.7f\n", lat, lon);
    else fprintf(log_f, ",\n");
    fflush(log_f);
}

static void send_estimate(const df_est_t *e, uint32_t seq) {
    df_do_msg_t d;
    double      t = now_s();

    memset(&d, 0, sizeof(d));
    memcpy(d.tag, "DFDO", 4);
    log_bearing(e);
    d.seq = seq;
    d.gate = e->gate;
    d.bad_ch = (int8_t)e->bad_ch;
    d.used_mask = (uint8_t)e->used;
    d.fit_pct = (uint8_t)lrint(fmin(1, fmax(0, e->doa.fit)) * 100);
    if (e->gate) {
        if (last_good_at < 0 || t - last_good_at >= HOLD_S) held_bearing = -1;     /* a signal anew */
        last_good_at = t;
        last_bearing = e->doa.bearing;
        last_mirror = e->doa.mirror;
        last_uncert = e->uncert_deg;
        if (e->doa.fit >= HOLD_FIT) {
            held_bearing = e->doa.bearing;
            held_mirror = e->doa.mirror;
            held_uncert = e->uncert_deg;
            held_fit = e->doa.fit;
        }
    }
    if (!e->gate && held_bearing >= 0) {    /* HOLD: the held one, its fit and +- */
        d.bearing_ddeg = ddeg(held_bearing);
        d.mirror_ddeg = ddeg(held_mirror);
        d.uncert_ddeg = clamp16(held_uncert * 10);
        d.fit_pct = (uint8_t)lrint(fmin(1, fmax(0, held_fit)) * 100);
    } else {
        d.bearing_ddeg = ddeg(last_bearing);
        d.mirror_ddeg = ddeg(last_mirror);
        d.uncert_ddeg = clamp16(last_uncert * 10);
    }
    d.level_ddb = clamp16(e->level_db * 10);
    d.snr_ddb = clamp16(e->doa.snr_db * 10);
    d.eig_ddb = clamp16(e->doa.eig_db * 10);
    d.papr_ddb = clamp16(e->doa.papr_db * 10);
    d.age_ds = last_good_at < 0 ? 0xffff : (uint16_t)fmin(65534, (t - last_good_at) * 10);
    d.npeaks = (uint8_t)(e->doa.npeaks > 3 ? 3 : e->doa.npeaks);
    d.nsrc = (uint8_t)(e->doa.nsrc > 0 ? e->doa.nsrc : 1);
    for (int i = 0; i < d.npeaks; i++) d.peaks_ddeg[i] = ddeg(e->doa.peaks[i]);
    for (int m = 0; m < nch && m < DF_PROTO_MAXCH; m++) {
        d.ch_ddb[m] = clamp16(e->ch_db[m] * 10);
        d.ch_clip_pm[m] = (uint8_t)fmin(255, e->clip[m] * 1000);
    }
    for (int g = 0; g < 360 && e->have_doa; g++) {     /* none: all 0 (too few antennas left) */
        double p = e->doa.p_db[g * DF_GRID / 360];

        d.p[g] = (uint8_t)fmax(0, fmin(255, 255 * (1 + p / 40)));
    }
    send_msg(&d, sizeof(d));
}

static void send_spectrum(uint32_t seq) {
    df_sp_msg_t s;

    memset(&s, 0, sizeof(s));
    memcpy(s.tag, "DFSP", 4);
    s.seq = seq;
    s.fc_hz = fc;
    s.rate = rate;
    s.floor_ddbfs = clamp16(eng.floor_db * 10);
    s.n = DF_PROTO_SPEC;
    for (int i = 0; i < DF_PROTO_SPEC; i++) {
        double v = fmax(eng.spec_db[2 * i], eng.spec_db[2 * i + 1]);

        s.db[i] = (uint8_t)fmax(0, fmin(255, (v - eng.floor_db + 20) * 2));
    }
    send_msg(&s, sizeof(s));
}

/* ------------------------------------------------------------------------------------------------- commands */

static bool parse_array(const char *a, df_array_t *out) {
    char kind[16];
    int  n, mm, used;

    if (sscanf(a, "%15s %d%n", kind, &n, &used) < 2 || n < 2 || n > DF_MAXCH) return false;
    a += used;
    if (!strcmp(kind, "uca") || !strcmp(kind, "ula")) {
        if (sscanf(a, "%d", &mm) != 1 || mm < 10 || mm > 20000) return false;
        if (kind[2] == 'a' && kind[1] == 'c') df_array_uca(out, n, mm / 1000.0);
        else df_array_ula(out, n, mm / 1000.0);
        return true;
    }
    if (!strcmp(kind, "custom")) {
        memset(out, 0, sizeof(*out));
        out->kind = DF_CUSTOM;
        out->n = n;
        for (int m = 0; m < n; m++) {
            int x, y;

            if (sscanf(a, "%d %d%n", &x, &y, &used) != 2) return false;
            a += used;
            out->x[m] = x / 1000.0;
            out->y[m] = y / 1000.0;
        }
        return true;
    }
    return false;
}

/* what the client asked: 0 nothing more, 1 calibrate (delays known), 2 open again (the rate), 3 quit */
static int apply(const char *c) {
    double     a = 0, b = 0;
    df_array_t na;

    switch (c[0]) {
        case 'F':                                       /* an IQ program's FREQ: the Kraken a quarter of the rate
                                                         * under it, the target on it (as page I's bands) */
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 24e6 && a <= 1766e6) {
                f_off = (int32_t)(rate / 4);
                fc = (uint32_t)(a - f_off);
                if (dev_is_open) dev_tune();
                eng.fc = fc;
                if (eng_on) df_engine_retarget(&eng, f_off);
                atomic_store(&iq_tuned, true);
                return 1;
            }
            break;
        case 'f':
            atomic_store(&iq_tuned, false);             /* the GUI tunes again */
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 24e6 && a <= 1766e6 && (uint32_t)a != fc) {
                fc = (uint32_t)a;
                if (dev_is_open) dev_tune();
                eng.fc = fc;
                if (eng_on) df_engine_retarget(&eng, f_off);
                return 1;
            }
            break;
        case 't':
            if (sscanf(c + 1, "%lf %lf", &a, &b) == 2 && fabs(a) < rate / 2.0 && b >= 1000 && b <= rate / 4.0) {
                bool newbw = (uint32_t)b != bw;

                f_off = (int32_t)a;
                bw = (uint32_t)b;
                if (newbw && dev_is_open) engine_setup();
                else if (eng_on) df_engine_retarget(&eng, f_off);
            }
            break;
        case 's':
            if (sscanf(c + 1, "%lf", &a) == 1 && (a == 250000 || a == 1024000 || a == 2048000 || a == 2400000) &&
                (uint32_t)a != rate) {
                rate = (uint32_t)a;
                if (fabs((double)f_off) > rate * 0.4) f_off = 0;
                return 2;
            }
            break;
        case 'g':
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 0 && a <= 500 && (int)a != gain) {
                gain = (int)a;
                if (dev_is_open) dev_gain();
                return 1;
            }
            break;
        case 'a':
            if (parse_array(c + 1, &na)) {
                arr = na;
                if (dev_is_open) engine_setup();
            }
            break;
        case 'm':
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 0 && a < DF_METHODS) eng.method = (df_method_t)(method = (int)a);
            break;
        case 'q':
            if (sscanf(c + 1, "%lf", &a) == 1) eng.sql_db = (sql = (int)a) / 10.0;
            break;
        case 'i':
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 0) eng.cal_every_s = (cal_every = (int)a) > 0 ? cal_every : -1;
            break;
        case 'k':
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 0 && a <= 3) eng.nsrc = nsrc_set = (int)a ? (int)a : -1;
            break;
        case 'l':
            if (sscanf(c + 1, "%lf", &a) == 1) {
                log_on = a != 0;
                if (!log_on && log_f) fclose(log_f), log_f = NULL;
            }
            break;
        case 'h':
            if (sscanf(c + 1, "%lf", &a) == 1) hand_heading = ((int)a % 360 + 360) % 360;
            break;
        case 'w':
            if (sscanf(c + 1, "%lf", &a) == 1) eng.fb = fb_on = a != 0;
            break;
        case 'o':
            if (sscanf(c + 1, "%lf", &a) == 1 && a >= 0 && a <= 3) eng.smooth = smooth_set = a >= 2 ? (int)a : 0;
            break;
        case '1':
        case '2':
        case '3':                                       /* another target: offset, bandwidth (0 off) */
            if (sscanf(c + 1, "%lf %lf", &a, &b) == 2 && fabs(a) <= 1.5e6 && b >= 0 && b <= 250000)
                tg_set(c[0] - '1', (int32_t)lrint(a), (uint32_t)lrint(b));
            break;
        case 'x':
            if (sscanf(c + 1, "%lf", &a) == 1) eng.user_off = user_off = (unsigned)a & 0xff;
            break;
        case 'c': return 1;
        case 'Q': return 3;
    }
    if (c[0] && strchr("amqkwox", c[0])) atomic_fetch_add(&set_gen, 1);   /* the other targets' engines anew */
    return 0;
}

/* ------------------------------------------------------------------------------------------------- main loop */

static void on_term(int s) {
    (void)s;
    atomic_store(&quit_now, true);
}

static void print_est(const df_est_t *e) {
    printf("%s lvl %5.1f  %s", st == DF_ST_TRACK ? "TRACK " : st == DF_ST_HOLD ? "HOLD  " : st == DF_ST_SLIP ? "SLIP  "
                               : st == DF_ST_LISTEN ? "LISTEN" : st == DF_ST_NOCAL ? "NOCAL " : "?     ",
           e->level_db, e->gate ? "" : "(");
    printf("brg %5.1f +-%4.1f fit %3.0f%% eig %4.1f", e->doa.bearing, e->uncert_deg, e->doa.fit * 100, e->doa.eig_db);
    printf("%s  ch", e->gate ? " " : ")");
    for (int m = 0; m < nch; m++) printf(" %c%4.1f", (e->used >> m) & 1 ? ' ' : '-', e->ch_db[m]);
    if (e->bad_ch >= 0) printf("  slip? CH%d", e->bad_ch);
    printf("\n");
    fflush(stdout);
}

/* Commands while nothing is open: only the settings taken, for the next open */
static void take_cmds_closed(void) {
    pthread_mutex_lock(&cmd_lock);
    for (int i = 0; i < ncmds; i++) (void)apply(cmds[i]);
    ncmds = 0;
    pthread_mutex_unlock(&cmd_lock);
}

/* Started by page I: its settings (replayed as soon as it connects) before the sticks open, so they open and calibrate
 * once, on its band, not first on the defaults; 4 s at most */
static void wait_settings(void) {
    for (int i = 0; i < 40 && !atomic_load(&quit_now); i++) {
        pthread_mutex_lock(&cmd_lock);
        bool settled = had_client && now_s() - cmd_t > 0.4;

        pthread_mutex_unlock(&cmd_lock);
        if (settled) break;
        usleep(100000);
    }
    take_cmds_closed();
}

static void run(bool print) {
    pin_self(cpu_engine, 0, "the engine");
    musb_irq_away();
    uint32_t seq = 0;
    double   last_st = 0, cpu_t = now_s(), cpu_wall = 0;
    struct timespec cpu0;

    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu0);
    while (!atomic_load(&quit_now)) {
        /* find it, open it */
        st = DF_ST_NODEV;
        why[0] = 0;
        while (!atomic_load(&quit_now) && !dev_there()) {
            take_cmds_closed();
            send_state();
            sleep(2);
        }
        if (on_its_own) take_cmds_closed();
        else wait_settings();
        if (atomic_load(&quit_now)) break;
        st = DF_ST_OPENING;
        send_state();
        have_cal = false;
        cal_at = -1;
        why[0] = note[0] = 0;
        if (dev_open()) {
            st = DF_ST_ERROR;
            logf_("%s", why);
            send_state();
            sleep(3);
            continue;
        }
        engine_setup();
        usleep(800000);                         /* the transfers' times for the first guess at the delays */
        {
            char b[160];
            int  n = 0;

            for (int m = 0; m < nch; m++) n += snprintf(b + n, sizeof(b) - (size_t)n, " %lld", (long long)wr_of(m));
            logf_("samples in so far:%s", b);
        }
        /* The X6200's USB host (MUSB: five receive endpoints, the Kraken's hub takes one) feeds four RTL streams at
         * most, B's included: a fifth one gets nothing at all (2026-11-17, any rate, whichever opened last). Those
         * that sent nothing are let go and shown GONE; the bearing comes from the rest (CH0, the noise source's,
         * opens first so it is never the one) */
        if (!synth) {
            int first0 = nch;

            for (int m = 0; m < nch; m++)
                if (wr_of(m) == 0 && first0 == nch) first0 = m;
            if (first0 < nch && first0 >= 2) {
                hw_drop_from(first0);
                snprintf(note, sizeof(note), "CH%d-%d sent nothing: the radio's USB feeds 4 RTLs, B's too",
                         first0, n_open - 1);
                logf_("%s", note);
                engine_setup();
            }
        }
        coarse_align();
        calibrate(true);

        double cal_try = now_s();

        bool reopen = false;

        while (!atomic_load(&quit_now) && !reopen) {
            /* commands */
            int todo = 0;

            pthread_mutex_lock(&cmd_lock);
            for (int i = 0; i < ncmds; i++) {
                int r = apply(cmds[i]);

                if (r > todo) todo = r;
            }
            ncmds = 0;
            pthread_mutex_unlock(&cmd_lock);
            if (todo == 3) {
                atomic_store(&quit_now, true);
                break;
            }
            if (todo == 2) {
                reopen = true;
                break;
            }
            if (todo == 1 || (!have_cal && now_s() - cal_try > 10)) {
                if (!have_cal) coarse_align();
                calibrate(!have_cal);
                cal_try = now_s();
                send_state();
            }

            /* gone? */
            for (int m = 0; m < nch; m++)
                if (atomic_load(&ch[m].ended) || now_s() - atomic_load(&ch[m].last_t) > 3) {
                    snprintf(why, sizeof(why), "CH%d (serial %.6s) stopped sending", m, ch[m].serial);
                    reopen = true;
                }
            if (reopen) break;

            /* a block */
            if (!wait_for(J, blk_n, 0.5)) continue;
            int64_t behind = wr_of(0) - J;

            if (behind > RING / 2) {            /* fell behind: start again at the newest */
                overruns++;
                J = wr_of(0) - blk_n;
                for (int m = 1; m < nch; m++)
                    if (J + kb[m] + blk_n > wr_of(m)) J = wr_of(m) - kb[m] - blk_n;
            }
            if (!read_at(J, blk_n, blk)) {
                J = wr_of(0) - blk_n;
                continue;
            }
            J += blk_n;
            atomic_store(&J_pub, J - blk_n);            /* the other targets read the same block */
            if (!eng_on) continue;

            df_est_t e;

            df_engine_block(&eng, (const uint8_t *const *)blk, blk_n, &e);
            last = e;
            double t = now_s();

            if (e.cal_wanted == 1) {
                slips++;
                slip_shown = true;
                if (e.bad_ch >= 0) logf_("slip on CH%d (%d so far): calibrating again", e.bad_ch, slips);
                else logf_("slip (%d so far): calibrating again", slips);
            }
            st = !have_cal ? DF_ST_NOCAL
               : slip_shown ? DF_ST_SLIP
               : e.gate ? DF_ST_TRACK
               : last_good_at >= 0 && t - last_good_at < HOLD_S ? DF_ST_HOLD
               : DF_ST_LISTEN;
            send_estimate(&e, seq);
            send_spectrum(seq);
            send_targets(seq);
            seq++;
            if (print) print_est(&e);
            if (e.cal_wanted) calibrate(e.cal_wanted == 1);

            /* the state twice a second, the CPU and gpsd's view once a second */
            if (t - last_st >= 0.5) {
                static int odd;

                last_st = t;
                send_state();
                if ((odd ^= 1)) send_gps();
            }
            if (t - cpu_t >= 1) {
                struct timespec c1;

                clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &c1);
                cpu_wall = (c1.tv_sec - cpu0.tv_sec) + (c1.tv_nsec - cpu0.tv_nsec) * 1e-9;
                cpu_pct = (int)lrint(100 * cpu_wall / (t - cpu_t));
                cpu0 = c1;
                cpu_t = t;
            }
        }
        atomic_store(&J_pub, -1);
        dev_close();
        if (reopen && !atomic_load(&quit_now)) {
            logf_("open again: %s", why[0] ? why : "a new rate");
            st = DF_ST_ERROR;
            send_state();
            sleep(1);
        }
    }
    tg_end_all();
    dev_close();
    st = DF_ST_NODEV;
    send_state();
    logf_("ended");
}

int main(int argc, char **argv) {
    const char *sock_name = DF_SOCK;
    bool        front = false, print = false;
    int         o, an = 5;
    double      a = 0, b = 0;

    df_array_uca(&arr, 5, 0.294);               /* the calculator at 433.92 MHz, s 0.5 */
    while ((o = getopt(argc, argv, "FS:pf:s:g:t:a:m:i:n:DT:C:I:")) != -1) {
        switch (o) {
            case 'F': front = true; break;
            case 'S': sock_name = optarg; break;
            case 'p': print = true; break;
            case 'f': fc = (uint32_t)atof(optarg); break;
            case 's': rate = (uint32_t)atof(optarg); break;
            case 'g': gain = atoi(optarg); break;
            case 't':
                if (sscanf(optarg, "%lf,%lf", &a, &b) >= 1) f_off = (int32_t)a;
                if (b > 0) bw = (uint32_t)b;
                break;
            case 'a': {
                char k[8];
                int  mm;

                if (sscanf(optarg, "%7[a-z],%d,%d", k, &an, &mm) != 3 || an < 2 || an > DF_MAXCH) return 1;
                if (!strcmp(k, "ula")) df_array_ula(&arr, an, mm / 1000.0);
                else df_array_uca(&arr, an, mm / 1000.0);
                break;
            }
            case 'm':
                method = atoi(optarg);
                if (method < 0 || method >= DF_METHODS) method = 0;
                break;
            case 'i': cal_every = atoi(optarg); break;
            case 'n':
                if ((n_open = atoi(optarg)) < 2 || n_open > NCH_KRAKEN) return 1;
                break;
            case 'D': dither_on = true; break;
            case 'I': iq_bw_set = atoi(optarg); break;
            case 'C':
                if (!strcmp(optarg, "off")) cores_on = false;
                else if (sscanf(optarg, "%d,%d,%d", &cpu_readers, &cpu_engine, &rt_prio) < 2) return 1;
                break;
            case 'T':
                synth = true;
                sscanf(optarg, "%lf,%lf,%d", &synth_bearing, &synth_rate, &synth_dead);
                break;
            default: return 1;
        }
    }

    /* the name first: a second copy stops here (Linux: an abstract name; elsewhere no socket, -p only) */
#ifdef __linux__
    struct sockaddr_un sa = {.sun_family = AF_UNIX};
    socklen_t          alen = (socklen_t)(offsetof(struct sockaddr_un, sun_path) + 1 + strlen(sock_name));

    if (strlen(sock_name) > sizeof(sa.sun_path) - 2) return 1;
    memcpy(sa.sun_path + 1, sock_name, strlen(sock_name));
    ls = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (ls < 0) return 1;
    if (bind(ls, (struct sockaddr *)&sa, alen) != 0) {
        printf("DF RUNNING\n");
        return 0;
    }
    listen(ls, 2);
#else
    (void)sock_name;
    front = true;
#endif
    on_its_own = front;
    if (!front) {
        pid_t p = fork();

        if (p < 0) return 1;
        if (p > 0) _exit(0);
        setsid();
        if ((p = fork()) < 0) return 1;
        if (p > 0) _exit(0);
        if (chdir("/") != 0) return 1;
        int nul = open("/dev/null", O_RDWR), lg = open("/tmp/x6x00-df.log", O_WRONLY | O_CREAT | O_APPEND, 0644);

        if (nul >= 0) dup2(nul, 0), dup2(nul, 1);
        if (lg >= 0) dup2(lg, 2);
    }
    setvbuf(stderr, NULL, _IOLBF, 0);

    struct sigaction s = {.sa_handler = on_term};

    sigemptyset(&s.sa_mask);
    sigaction(SIGTERM, &s, NULL);
    sigaction(SIGINT, &s, NULL);
    signal(SIGPIPE, SIG_IGN);
    for (int m = 0; m < DF_MAXCH; m++)
        if (!(ch[m].ring = malloc(2 * (size_t)RING))) return 1;

    pthread_t th;
    bool      th_on = ls >= 0 && pthread_create(&th, NULL, sock_thread, NULL) == 0;
    pthread_t gth;

    iq_start();                         /* Heimdall's IQ server, for other programs */
    df_gps_init(&gps);
    if (pthread_create(&gth, NULL, gps_thread, NULL) == 0) pthread_detach(gth);     /* gpsd, when it runs */

    logf_("started%s: %u Hz, target %+d Hz / %u Hz, %u S/s", synth ? " (synthetic)" : "", fc, f_off, bw, rate);
    run(print);
    if (th_on) pthread_join(th, NULL);
    return 0;
}
