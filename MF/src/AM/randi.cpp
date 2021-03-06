//
// File: randi.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 10-Dec-2017 18:29:31
//

// Include Files
#include "rt_nonfinite.h"
#include "AM.h"
#include "randi.h"
#include "rand.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : double
//
double randi()
{
  double r;
  r = b_rand();
  return 1.0 + floor(r * 100.0);
}

//
// File trailer for randi.cpp
//
// [EOF]
//
