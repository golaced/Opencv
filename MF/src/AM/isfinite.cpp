//
// File: isfinite.cpp
//
// MATLAB Coder version            : 3.0
// C/C++ source code generated on  : 10-Dec-2017 18:29:31
//

// Include Files
#include "rt_nonfinite.h"
#include "AM.h"
#include "isfinite.h"

// Function Definitions

//
// Arguments    : double x
// Return Type  : boolean_T
//
boolean_T b_isfinite(double x)
{
  return (!rtIsInf(x)) && (!rtIsNaN(x));
}

//
// File trailer for isfinite.cpp
//
// [EOF]
//
