/*
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  x6x00-df's position and heading from gpsd (ROADMAP.md, 4): gpsd's JSON reports read one line at a time (TPV:
 *  the fix, speed and course; ATT: a compass's heading), and which heading the bearings take. No hardware and no
 *  socket here, so the host checks run it (df_test.c); df.c talks to gpsd.
 *
 *  The heading, best first:
 *    - a compass behind gpsd (ATT "heading"), while its reports keep coming (3 s);
 *    - the course over the ground while moving at DF_GPS_MOVING m/s or more (a vehicle with the array fixed to it,
 *      ANT 0 forward), and the last such course for DF_GPS_HOLD_S after it slows (stopped at a light, it has not
 *      turned);
 *    - none: the heading set by hand on page I.
 */
#pragma once

#include <stdbool.h>

#define DF_GPS_MOVING   2.0     /* m/s: under it the course is noise */
#define DF_GPS_HOLD_S   120.0   /* the last course kept this long after slowing */
#define DF_GPS_FRESH_S  3.0     /* a report older than this counts as gone */

enum { DF_HEAD_NONE = 0, DF_HEAD_COURSE, DF_HEAD_COMPASS };

typedef struct {
    int    mode;                /* gpsd's fix: 0-1 none, 2 2D, 3 3D */
    double lat, lon;            /* degrees */
    double speed;               /* m/s, -1 unknown */
    double tpv_at;              /* our clock, s; 0 none yet */
    double compass;             /* ATT heading, degrees; -1 none */
    double compass_at;
    double course;              /* the last course over the ground taken while moving; -1 none */
    double course_at;
} df_gps_t;

void df_gps_init(df_gps_t *g);
/* One line from gpsd (a JSON object); anything not a TPV or ATT report is passed over. now: our clock, s */
void df_gps_line(df_gps_t *g, const char *line, double now);
/* The heading the bearings take now: DF_HEAD_* and the degrees in *deg */
int df_gps_heading(const df_gps_t *g, double now, double *deg);
/* A fix to put in the log: true and the position */
bool df_gps_fix(const df_gps_t *g, double now, double *lat, double *lon);
