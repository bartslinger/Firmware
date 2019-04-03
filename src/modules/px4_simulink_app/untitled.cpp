//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: untitled.cpp
//
// Code generated for Simulink model 'untitled'.
//
// Model version                  : 1.1
// Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
// C/C++ source code generated on : Thu Mar 28 17:46:12 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "untitled.h"
#include "untitled_private.h"

// Block states (default storage)
DW_untitled_T untitled_DW;

// Real-time model
RT_MODEL_untitled_T untitled_M_;
RT_MODEL_untitled_T *const untitled_M = &untitled_M_;

// Forward declaration for local functions
static void untitled_SystemCore_release(const px4_internal_block_Subscriber_T
  *obj);
static void untitled_SystemCore_delete(const px4_internal_block_Subscriber_T
  *obj);
static void matlabCodegenHandle_matlabCodeg(px4_internal_block_Subscriber_T *obj);
static void untitled_SystemCore_release(const px4_internal_block_Subscriber_T
  *obj)
{
  if ((obj->isInitialized == 1) && obj->isSetupComplete) {
    uORB_read_terminate(&obj->eventStructObj);
  }
}

static void untitled_SystemCore_delete(const px4_internal_block_Subscriber_T
  *obj)
{
  untitled_SystemCore_release(obj);
}

static void matlabCodegenHandle_matlabCodeg(px4_internal_block_Subscriber_T *obj)
{
  if (!obj->matlabCodegenIsDeleted) {
    obj->matlabCodegenIsDeleted = true;
    untitled_SystemCore_delete(obj);
  }
}

// Model step function
void untitled_step(void)
{
  px4_Bus_battery_status b_varargout_2;

  // MATLABSystem: '<S2>/SourceBlock'
  uORB_read_step(untitled_DW.obj.orbMetadataObj, &untitled_DW.obj.eventStructObj,
                 &b_varargout_2, false, 1.0);
}

// Model initialize function
void untitled_initialize(void)
{
  // Registration code

  // initialize error status
  rtmSetErrorStatus(untitled_M, (NULL));

  // states (dwork)
  (void) memset((void *)&untitled_DW, 0,
                sizeof(DW_untitled_T));

  // Start for MATLABSystem: '<S2>/SourceBlock'
  untitled_DW.obj.matlabCodegenIsDeleted = true;
  untitled_DW.obj.isInitialized = 0;
  untitled_DW.obj.ticksUntilNextHit = 0.0;
  untitled_DW.obj.matlabCodegenIsDeleted = false;
  untitled_DW.obj.isSetupComplete = false;
  untitled_DW.obj.isInitialized = 1;
  untitled_DW.obj.orbMetadataObj = ORB_ID(battery_status);
  uORB_read_initialize(untitled_DW.obj.orbMetadataObj,
                       &untitled_DW.obj.eventStructObj, 4.0);
  untitled_DW.obj.isSetupComplete = true;
}

// Model terminate function
void untitled_terminate(void)
{
  // Terminate for MATLABSystem: '<S2>/SourceBlock'
  matlabCodegenHandle_matlabCodeg(&untitled_DW.obj);
}

//
// File trailer for generated code.
//
// [EOF]
//
