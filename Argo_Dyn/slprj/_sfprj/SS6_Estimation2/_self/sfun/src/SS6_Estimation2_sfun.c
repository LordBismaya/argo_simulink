/* Include files */

#include "SS6_Estimation2_sfun.h"
#include "SS6_Estimation2_sfun_debug_macros.h"
#include "c1_SS6_Estimation2.h"
#include "c2_SS6_Estimation2.h"
#include "c3_SS6_Estimation2.h"
#include "c4_SS6_Estimation2.h"
#include "c5_SS6_Estimation2.h"
#include "c6_SS6_Estimation2.h"
#include "c7_SS6_Estimation2.h"
#include "c8_SS6_Estimation2.h"
#include "c9_SS6_Estimation2.h"
#include "c10_SS6_Estimation2.h"
#include "c11_SS6_Estimation2.h"
#include "c12_SS6_Estimation2.h"
#include "c13_SS6_Estimation2.h"
#include "c14_SS6_Estimation2.h"
#include "c15_SS6_Estimation2.h"
#include "c16_SS6_Estimation2.h"
#include "c17_SS6_Estimation2.h"
#include "c18_SS6_Estimation2.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */
uint32_T _SS6_Estimation2MachineNumber_;

/* Function Declarations */

/* Function Definitions */
void SS6_Estimation2_initializer(void)
{
}

void SS6_Estimation2_terminator(void)
{
}

/* SFunction Glue Code */
unsigned int sf_SS6_Estimation2_method_dispatcher(SimStruct *simstructPtr,
  unsigned int chartFileNumber, const char* specsCksum, int_T method, void *data)
{
  if (chartFileNumber==1) {
    c1_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==2) {
    c2_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==3) {
    c3_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==4) {
    c4_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==5) {
    c5_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==6) {
    c6_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==7) {
    c7_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==8) {
    c8_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==9) {
    c9_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==10) {
    c10_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==11) {
    c11_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==12) {
    c12_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==13) {
    c13_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==14) {
    c14_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==15) {
    c15_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==16) {
    c16_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==17) {
    c17_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  if (chartFileNumber==18) {
    c18_SS6_Estimation2_method_dispatcher(simstructPtr, method, data);
    return 1;
  }

  return 0;
}

extern void sf_SS6_Estimation2_uses_exported_functions(int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[])
{
  plhs[0] = mxCreateLogicalScalar(0);
}

unsigned int sf_SS6_Estimation2_process_check_sum_call( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[20];
  if (nrhs<1 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the checksum */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"sf_get_check_sum"))
    return 0;
  plhs[0] = mxCreateDoubleMatrix( 1,4,mxREAL);
  if (nrhs>1 && mxIsChar(prhs[1])) {
    mxGetString(prhs[1], commandName,sizeof(commandName)/sizeof(char));
    commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
    if (!strcmp(commandName,"machine")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2799772826U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4156275511U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(259194220U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(4134356258U);
    } else if (!strcmp(commandName,"exportedFcn")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0U);
    } else if (!strcmp(commandName,"makefile")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1612030183U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3133317487U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(4140863094U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(764304773U);
    } else if (nrhs==3 && !strcmp(commandName,"chart")) {
      unsigned int chartFileNumber;
      chartFileNumber = (unsigned int)mxGetScalar(prhs[2]);
      switch (chartFileNumber) {
       case 1:
        {
          extern void sf_c1_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c1_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 2:
        {
          extern void sf_c2_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c2_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 3:
        {
          extern void sf_c3_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c3_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 4:
        {
          extern void sf_c4_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c4_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 5:
        {
          extern void sf_c5_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c5_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 6:
        {
          extern void sf_c6_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c6_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 7:
        {
          extern void sf_c7_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c7_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 8:
        {
          extern void sf_c8_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c8_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 9:
        {
          extern void sf_c9_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c9_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 10:
        {
          extern void sf_c10_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c10_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 11:
        {
          extern void sf_c11_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c11_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 12:
        {
          extern void sf_c12_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c12_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 13:
        {
          extern void sf_c13_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c13_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 14:
        {
          extern void sf_c14_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c14_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 15:
        {
          extern void sf_c15_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c15_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 16:
        {
          extern void sf_c16_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c16_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 17:
        {
          extern void sf_c17_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c17_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       case 18:
        {
          extern void sf_c18_SS6_Estimation2_get_check_sum(mxArray *plhs[]);
          sf_c18_SS6_Estimation2_get_check_sum(plhs);
          break;
        }

       default:
        ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(0.0);
        ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(0.0);
      }
    } else if (!strcmp(commandName,"target")) {
      ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3061339410U);
      ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1991824845U);
      ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3599338742U);
      ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2357874978U);
    } else {
      return 0;
    }
  } else {
    ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2039378828U);
    ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2227048543U);
    ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(3253555085U);
    ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1139672469U);
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SS6_Estimation2_autoinheritance_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[32];
  char aiChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]) )
    return 0;

  /* Possible call to get the autoinheritance_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_autoinheritance_info"))
    return 0;
  mxGetString(prhs[2], aiChksum,sizeof(aiChksum)/sizeof(char));
  aiChksum[(sizeof(aiChksum)/sizeof(char)-1)] = '\0';

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(aiChksum, "hbBb04f5OgypQJURcTJOGE") == 0) {
          extern mxArray *sf_c1_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c1_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 2:
      {
        if (strcmp(aiChksum, "OquLqqtMYYVdqCWKjMkOLH") == 0) {
          extern mxArray *sf_c2_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c2_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 3:
      {
        if (strcmp(aiChksum, "Jb33eHdHaBoeR9W53vjoFH") == 0) {
          extern mxArray *sf_c3_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c3_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 4:
      {
        if (strcmp(aiChksum, "N8dAdA9bC8D9bD6TucC9ZH") == 0) {
          extern mxArray *sf_c4_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c4_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 5:
      {
        if (strcmp(aiChksum, "H1lG8enhiR6wHkR31rX7ND") == 0) {
          extern mxArray *sf_c5_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c5_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 6:
      {
        if (strcmp(aiChksum, "o2G06cpO3M4BRVz1MzVKGD") == 0) {
          extern mxArray *sf_c6_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c6_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 7:
      {
        if (strcmp(aiChksum, "XtLWDp4g3cmASKsPhW5TVH") == 0) {
          extern mxArray *sf_c7_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c7_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 8:
      {
        if (strcmp(aiChksum, "y1cG55g0wEVBwWeaZX6MVB") == 0) {
          extern mxArray *sf_c8_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c8_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 9:
      {
        if (strcmp(aiChksum, "CPKQFsCKpeXI1ANyxjqOkH") == 0) {
          extern mxArray *sf_c9_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c9_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 10:
      {
        if (strcmp(aiChksum, "vRU7xjquqLOo3QxzmQF1fH") == 0) {
          extern mxArray *sf_c10_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c10_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 11:
      {
        if (strcmp(aiChksum, "GjkT5x53qTcAfKHkAtG5g") == 0) {
          extern mxArray *sf_c11_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c11_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 12:
      {
        if (strcmp(aiChksum, "zDlWNo2keAydZ3PXx6Gl9B") == 0) {
          extern mxArray *sf_c12_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c12_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 13:
      {
        if (strcmp(aiChksum, "h5vdpLPXeGEihSyydS0rGC") == 0) {
          extern mxArray *sf_c13_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c13_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 14:
      {
        if (strcmp(aiChksum, "XtLWDp4g3cmASKsPhW5TVH") == 0) {
          extern mxArray *sf_c14_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c14_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 15:
      {
        if (strcmp(aiChksum, "Jb33eHdHaBoeR9W53vjoFH") == 0) {
          extern mxArray *sf_c15_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c15_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 16:
      {
        if (strcmp(aiChksum, "o2G06cpO3M4BRVz1MzVKGD") == 0) {
          extern mxArray *sf_c16_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c16_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 17:
      {
        if (strcmp(aiChksum, "N8dAdA9bC8D9bD6TucC9ZH") == 0) {
          extern mxArray *sf_c17_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c17_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     case 18:
      {
        if (strcmp(aiChksum, "88AzXH0rGLUbpKdAlPYK4F") == 0) {
          extern mxArray *sf_c18_SS6_Estimation2_get_autoinheritance_info(void);
          plhs[0] = sf_c18_SS6_Estimation2_get_autoinheritance_info();
          break;
        }

        plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SS6_Estimation2_get_eml_resolved_functions_info( int nlhs,
  mxArray * plhs[], int nrhs, const mxArray * prhs[] )
{

#ifdef MATLAB_MEX_FILE

  char commandName[64];
  if (nrhs<2 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the get_eml_resolved_functions_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_eml_resolved_functions_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        extern const mxArray
          *sf_c1_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c1_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 2:
      {
        extern const mxArray
          *sf_c2_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c2_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 3:
      {
        extern const mxArray
          *sf_c3_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c3_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 4:
      {
        extern const mxArray
          *sf_c4_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c4_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 5:
      {
        extern const mxArray
          *sf_c5_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c5_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 6:
      {
        extern const mxArray
          *sf_c6_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c6_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 7:
      {
        extern const mxArray
          *sf_c7_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c7_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 8:
      {
        extern const mxArray
          *sf_c8_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c8_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 9:
      {
        extern const mxArray
          *sf_c9_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c9_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 10:
      {
        extern const mxArray
          *sf_c10_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c10_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 11:
      {
        extern const mxArray
          *sf_c11_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c11_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 12:
      {
        extern const mxArray
          *sf_c12_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c12_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 13:
      {
        extern const mxArray
          *sf_c13_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c13_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 14:
      {
        extern const mxArray
          *sf_c14_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c14_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 15:
      {
        extern const mxArray
          *sf_c15_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c15_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 16:
      {
        extern const mxArray
          *sf_c16_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c16_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 17:
      {
        extern const mxArray
          *sf_c17_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c17_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     case 18:
      {
        extern const mxArray
          *sf_c18_SS6_Estimation2_get_eml_resolved_functions_info(void);
        mxArray *persistentMxArray = (mxArray *)
          sf_c18_SS6_Estimation2_get_eml_resolved_functions_info();
        plhs[0] = mxDuplicateArray(persistentMxArray);
        mxDestroyArray(persistentMxArray);
        break;
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;

#else

  return 0;

#endif

}

unsigned int sf_SS6_Estimation2_third_party_uses_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the third_party_uses_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_third_party_uses_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "LZfOmqNraPNCSz3iJKadoG") == 0) {
          extern mxArray *sf_c1_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c1_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "ApqOx7cH1YhKzqHE0pnSK") == 0) {
          extern mxArray *sf_c2_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c2_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c3_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c3_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c4_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c4_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "BHp0yp4DBJb5sLMdig0kJG") == 0) {
          extern mxArray *sf_c5_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c5_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c6_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c6_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c7_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c7_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "45vR29wOiaO707YVLpykwF") == 0) {
          extern mxArray *sf_c8_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c8_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "V4Mqh9ml3CUa10H2afdBPE") == 0) {
          extern mxArray *sf_c9_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c9_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "F7ISXg0e6e4OH33X2IsQjD") == 0) {
          extern mxArray *sf_c10_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c10_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "xnxTHjTTzdqlhD4f2mlq5B") == 0) {
          extern mxArray *sf_c11_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c11_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "184ndso7wkSNklMCeD2XSF") == 0) {
          extern mxArray *sf_c12_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c12_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "QDYDQhajYAgbxmz2rwNjIC") == 0) {
          extern mxArray *sf_c13_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c13_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c14_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c14_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 15:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c15_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c15_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 16:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c16_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c16_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 17:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c17_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c17_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     case 18:
      {
        if (strcmp(tpChksum, "sW8JcKYBxwefAdqhL8QcF") == 0) {
          extern mxArray *sf_c18_SS6_Estimation2_third_party_uses_info(void);
          plhs[0] = sf_c18_SS6_Estimation2_third_party_uses_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_SS6_Estimation2_jit_fallback_info( int nlhs, mxArray * plhs[],
  int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the jit_fallback_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_jit_fallback_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "LZfOmqNraPNCSz3iJKadoG") == 0) {
          extern mxArray *sf_c1_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c1_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "ApqOx7cH1YhKzqHE0pnSK") == 0) {
          extern mxArray *sf_c2_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c2_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c3_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c3_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c4_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c4_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "BHp0yp4DBJb5sLMdig0kJG") == 0) {
          extern mxArray *sf_c5_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c5_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c6_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c6_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c7_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c7_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "45vR29wOiaO707YVLpykwF") == 0) {
          extern mxArray *sf_c8_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c8_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "V4Mqh9ml3CUa10H2afdBPE") == 0) {
          extern mxArray *sf_c9_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c9_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "F7ISXg0e6e4OH33X2IsQjD") == 0) {
          extern mxArray *sf_c10_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c10_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "xnxTHjTTzdqlhD4f2mlq5B") == 0) {
          extern mxArray *sf_c11_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c11_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "184ndso7wkSNklMCeD2XSF") == 0) {
          extern mxArray *sf_c12_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c12_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "QDYDQhajYAgbxmz2rwNjIC") == 0) {
          extern mxArray *sf_c13_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c13_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c14_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c14_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 15:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c15_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c15_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 16:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c16_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c16_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 17:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c17_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c17_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     case 18:
      {
        if (strcmp(tpChksum, "sW8JcKYBxwefAdqhL8QcF") == 0) {
          extern mxArray *sf_c18_SS6_Estimation2_jit_fallback_info(void);
          plhs[0] = sf_c18_SS6_Estimation2_jit_fallback_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

unsigned int sf_SS6_Estimation2_updateBuildInfo_args_info( int nlhs, mxArray *
  plhs[], int nrhs, const mxArray * prhs[] )
{
  char commandName[64];
  char tpChksum[64];
  if (nrhs<3 || !mxIsChar(prhs[0]))
    return 0;

  /* Possible call to get the updateBuildInfo_args_info */
  mxGetString(prhs[0], commandName,sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  mxGetString(prhs[2], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName,"get_updateBuildInfo_args_info"))
    return 0;

  {
    unsigned int chartFileNumber;
    chartFileNumber = (unsigned int)mxGetScalar(prhs[1]);
    switch (chartFileNumber) {
     case 1:
      {
        if (strcmp(tpChksum, "LZfOmqNraPNCSz3iJKadoG") == 0) {
          extern mxArray *sf_c1_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c1_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 2:
      {
        if (strcmp(tpChksum, "ApqOx7cH1YhKzqHE0pnSK") == 0) {
          extern mxArray *sf_c2_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c2_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 3:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c3_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c3_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 4:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c4_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c4_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 5:
      {
        if (strcmp(tpChksum, "BHp0yp4DBJb5sLMdig0kJG") == 0) {
          extern mxArray *sf_c5_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c5_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 6:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c6_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c6_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 7:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c7_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c7_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 8:
      {
        if (strcmp(tpChksum, "45vR29wOiaO707YVLpykwF") == 0) {
          extern mxArray *sf_c8_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c8_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 9:
      {
        if (strcmp(tpChksum, "V4Mqh9ml3CUa10H2afdBPE") == 0) {
          extern mxArray *sf_c9_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c9_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 10:
      {
        if (strcmp(tpChksum, "F7ISXg0e6e4OH33X2IsQjD") == 0) {
          extern mxArray *sf_c10_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c10_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 11:
      {
        if (strcmp(tpChksum, "xnxTHjTTzdqlhD4f2mlq5B") == 0) {
          extern mxArray *sf_c11_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c11_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 12:
      {
        if (strcmp(tpChksum, "184ndso7wkSNklMCeD2XSF") == 0) {
          extern mxArray *sf_c12_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c12_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 13:
      {
        if (strcmp(tpChksum, "QDYDQhajYAgbxmz2rwNjIC") == 0) {
          extern mxArray *sf_c13_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c13_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 14:
      {
        if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
          extern mxArray *sf_c14_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c14_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 15:
      {
        if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
          extern mxArray *sf_c15_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c15_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 16:
      {
        if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
          extern mxArray *sf_c16_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c16_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 17:
      {
        if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
          extern mxArray *sf_c17_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c17_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     case 18:
      {
        if (strcmp(tpChksum, "sW8JcKYBxwefAdqhL8QcF") == 0) {
          extern mxArray *sf_c18_SS6_Estimation2_updateBuildInfo_args_info(void);
          plhs[0] = sf_c18_SS6_Estimation2_updateBuildInfo_args_info();
          break;
        }
      }

     default:
      plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
  }

  return 1;
}

void sf_SS6_Estimation2_get_post_codegen_info( int nlhs, mxArray * plhs[], int
  nrhs, const mxArray * prhs[] )
{
  unsigned int chartFileNumber = (unsigned int) mxGetScalar(prhs[0]);
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  switch (chartFileNumber) {
   case 1:
    {
      if (strcmp(tpChksum, "LZfOmqNraPNCSz3iJKadoG") == 0) {
        extern mxArray *sf_c1_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c1_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 2:
    {
      if (strcmp(tpChksum, "ApqOx7cH1YhKzqHE0pnSK") == 0) {
        extern mxArray *sf_c2_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c2_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 3:
    {
      if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
        extern mxArray *sf_c3_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c3_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 4:
    {
      if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
        extern mxArray *sf_c4_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c4_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 5:
    {
      if (strcmp(tpChksum, "BHp0yp4DBJb5sLMdig0kJG") == 0) {
        extern mxArray *sf_c5_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c5_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 6:
    {
      if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
        extern mxArray *sf_c6_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c6_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 7:
    {
      if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
        extern mxArray *sf_c7_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c7_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 8:
    {
      if (strcmp(tpChksum, "45vR29wOiaO707YVLpykwF") == 0) {
        extern mxArray *sf_c8_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c8_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 9:
    {
      if (strcmp(tpChksum, "V4Mqh9ml3CUa10H2afdBPE") == 0) {
        extern mxArray *sf_c9_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c9_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 10:
    {
      if (strcmp(tpChksum, "F7ISXg0e6e4OH33X2IsQjD") == 0) {
        extern mxArray *sf_c10_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c10_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 11:
    {
      if (strcmp(tpChksum, "xnxTHjTTzdqlhD4f2mlq5B") == 0) {
        extern mxArray *sf_c11_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c11_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 12:
    {
      if (strcmp(tpChksum, "184ndso7wkSNklMCeD2XSF") == 0) {
        extern mxArray *sf_c12_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c12_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 13:
    {
      if (strcmp(tpChksum, "QDYDQhajYAgbxmz2rwNjIC") == 0) {
        extern mxArray *sf_c13_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c13_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 14:
    {
      if (strcmp(tpChksum, "RjoCiBM5jPheWLwnBtCDIE") == 0) {
        extern mxArray *sf_c14_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c14_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 15:
    {
      if (strcmp(tpChksum, "VDOXVMN2OffVJVx1mVOexG") == 0) {
        extern mxArray *sf_c15_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c15_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 16:
    {
      if (strcmp(tpChksum, "nrpenqkF8ejDanyXb4eC") == 0) {
        extern mxArray *sf_c16_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c16_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 17:
    {
      if (strcmp(tpChksum, "ksUZsNZC3IraATfXwR1QqE") == 0) {
        extern mxArray *sf_c17_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c17_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   case 18:
    {
      if (strcmp(tpChksum, "sW8JcKYBxwefAdqhL8QcF") == 0) {
        extern mxArray *sf_c18_SS6_Estimation2_get_post_codegen_info(void);
        plhs[0] = sf_c18_SS6_Estimation2_get_post_codegen_info();
        return;
      }
    }
    break;

   default:
    break;
  }

  plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
}

void SS6_Estimation2_debug_initialize(struct SfDebugInstanceStruct*
  debugInstance)
{
  _SS6_Estimation2MachineNumber_ = sf_debug_initialize_machine(debugInstance,
    "SS6_Estimation2","sfun",0,18,0,0,0);
  sf_debug_set_machine_event_thresholds(debugInstance,
    _SS6_Estimation2MachineNumber_,0,0);
  sf_debug_set_machine_data_thresholds(debugInstance,
    _SS6_Estimation2MachineNumber_,0);
}

void SS6_Estimation2_register_exported_symbols(SimStruct* S)
{
}

static mxArray* sRtwOptimizationInfoStruct= NULL;
mxArray* load_SS6_Estimation2_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct==NULL) {
    sRtwOptimizationInfoStruct = sf_load_rtw_optimization_info("SS6_Estimation2",
      "SS6_Estimation2");
    mexMakeArrayPersistent(sRtwOptimizationInfoStruct);
  }

  return(sRtwOptimizationInfoStruct);
}

void unload_SS6_Estimation2_optimization_info(void)
{
  if (sRtwOptimizationInfoStruct!=NULL) {
    mxDestroyArray(sRtwOptimizationInfoStruct);
    sRtwOptimizationInfoStruct = NULL;
  }
}
