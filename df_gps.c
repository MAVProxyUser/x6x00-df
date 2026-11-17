/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  x6x00-df's position and heading from gpsd's reports (df_gps.h). gpsd sends one JSON object a line; only a few
 *  numbers of two classes are wanted, so they are picked out by key rather than parsed whole.
 */
#include "df_gps.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void df_gps_init(df_gps_t *g) {
    memset(g, 0, sizeof(*g));
    g->speed = -1;
    g->compass = -1;
    g->course = -1;
}

/* The number after "key": in a JSON object (spaces allowed round the colon); false when the key is not there or holds
 * no number (null, a string) */
static bool num(const char *line, const char *key, double *v) {
    char        k[32];
    const char *p = line;
    size_t      n;

    n = (size_t)snprintf(k, sizeof(k), "\"%s\"", key);
    if (n >= sizeof(k)) return false;
    while ((p = strstr(p, k))) {
        const char *q = p + n;
        char       *end;

        while (*q == ' ' || *q == '\t') q++;
        if (*q != ':') {
            p += n;
            continue;
        }
        q++;
        while (*q == ' ' || *q == '\t') q++;
        *v = strtod(q, &end);
        return end != q && isfinite(*v);
    }
    return false;
}

static bool is_class(const char *line, const char *cls) {
    char        k[24];
    const char *p = strstr(line, "\"class\"");

    if (!p) return false;
    p += 7;
    while (*p == ' ' || *p == '\t') p++;
    if (*p++ != ':') return false;
    while (*p == ' ' || *p == '\t') p++;
    snprintf(k, sizeof(k), "\"%s\"", cls);
    return !strncmp(p, k, strlen(k));
}

static double wrap360(double d) {
    d = fmod(d, 360.0);
    return d < 0 ? d + 360.0 : d;
}

void df_gps_line(df_gps_t *g, const char *line, double now) {
    double v;

    if (is_class(line, "TPV")) {
        g->mode = num(line, "mode", &v) ? (int)v : 0;
        if (g->mode >= 2 && num(line, "lat", &v)) g->lat = v;
        if (g->mode >= 2 && num(line, "lon", &v)) g->lon = v;
        g->speed = num(line, "speed", &v) ? v : -1;
        if (g->mode >= 2 && g->speed >= DF_GPS_MOVING && num(line, "track", &v)) {
            g->course = wrap360(v);
            g->course_at = now;
        }
        g->tpv_at = now;
    } else if (is_class(line, "ATT")) {
        if (num(line, "heading", &v)) {
            g->compass = wrap360(v);
            g->compass_at = now;
        }
    }
}

int df_gps_heading(const df_gps_t *g, double now, double *deg) {
    if (g->compass >= 0 && now - g->compass_at <= DF_GPS_FRESH_S) {
        *deg = g->compass;
        return DF_HEAD_COMPASS;
    }
    if (g->course >= 0 && g->tpv_at > 0 && now - g->tpv_at <= DF_GPS_FRESH_S && g->mode >= 2 &&
        now - g->course_at <= DF_GPS_HOLD_S) {
        *deg = g->course;
        return DF_HEAD_COURSE;
    }
    return DF_HEAD_NONE;
}

bool df_gps_fix(const df_gps_t *g, double now, double *lat, double *lon) {
    if (g->mode < 2 || g->tpv_at <= 0 || now - g->tpv_at > DF_GPS_FRESH_S) return false;
    *lat = g->lat;
    *lon = g->lon;
    return true;
}
