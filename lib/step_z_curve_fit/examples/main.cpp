//
// Classroom License -- for classroom instructional use only.  Not for
// government, commercial, academic research, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.6
// C/C++ source code generated on  : 30-Jun-2023 18:09:54
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include Files
#include "main.h"
#include "rt_nonfinite.h"
#include "step_z_curve_fit.h"
#include "step_z_curve_fit_terminate.h"
#include "coder_array.h"

// Function Declarations
static void argInit_1x2_real_T(double result[2]);

static coder::array<double, 2U> argInit_1xUnbounded_real_T();

static double argInit_real_T();

// Function Definitions
//
// Arguments    : double result[2]
// Return Type  : void
//
static void argInit_1x2_real_T(double result[2])
{
  // Loop over the array to initialize each element.
  for (int idx1{0}; idx1 < 2; idx1++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx1] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : coder::array<double, 2U>
//
static coder::array<double, 2U> argInit_1xUnbounded_real_T()
{
  coder::array<double, 2U> result;
  // Set the size of the array.
  // Change this size to the value that the application requires.
  result.set_size(1, 310);
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 1; idx0++) {
    for (int idx1{0}; idx1 < result.size(1); idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx1] = argInit_real_T();
    }
  }
  return result;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : int argc
//                char **argv
// Return Type  : int
//
int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_step_z_curve_fit();
  // Terminate the application.
  // You do not need to do this more than one time.
  step_z_curve_fit_terminate();
  return 0;
}

//
// Arguments    : void
// Return Type  : void
//
void main_step_z_curve_fit()
{
  coder::array<double, 2U> xdata_tmp;
  double x0_tmp[2];
  double AH;
  double end_time;
  // Initialize function 'step_z_curve_fit' input arguments.
  // Initialize function input argument 'xdata'.
  xdata_tmp = argInit_1xUnbounded_real_T();
  // Initialize function input argument 'ydata'.
  // Initialize function input argument 'x0'.
  argInit_1x2_real_T(x0_tmp);
  // Initialize function input argument 'lb'.
  // Initialize function input argument 'ub'.
  // Call the entry-point 'step_z_curve_fit'.
  step_z_curve_fit(xdata_tmp, xdata_tmp, x0_tmp, x0_tmp, x0_tmp, &AH,
                   &end_time);
}

//
// File trailer for main.cpp
//
// [EOF]
//
