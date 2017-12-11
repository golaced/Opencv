//
// File: AM.h
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 10-Dec-2017 18:29:31
//
#ifndef __AM_H__
#define __AM_H__

// Include Files
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "AM_types.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
using namespace std;
// Function Declarations
extern void AM(float map[500], float camera[4], short *first_check_id, const
               float c1[5], const float c2[5], const float c3[5], const float
               c4[5], const float c5[5], const float c6[5], short cloud_size,
               float mark_pos_ero_check, short lock_cnt, float center_fix_flt,
               short *init, short cal_sel);
extern void c_mark_pos_lock_cnt_not_empty_i();
extern void mark_buf_cnt_not_empty_init();
extern void mark_pos_map_buf_not_empty_init();

#endif

//
// File trailer for AM.h
//
// [EOF]
//
