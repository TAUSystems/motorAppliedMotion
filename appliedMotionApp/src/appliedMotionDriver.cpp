/*
FILENAME... StepperDriver.cpp
USAGE...    Motor driver support for the Applied Motion Stepper controllers
            Includes all ST5/10, STF, STM, SWM, STAC5 and STAC6 drives

Adapted from ACS MCB-4B driver by Mark Rivers

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>
#include "StepperDriver.h"

#define NINT(f) (int)((f)>0 ? (f)+0.5 : (f)-0.5)

// Overload AsynController::writeReadController(). 
// 
// TCP messages to this controller need to start with a packet header 0x00 0x07.
// Overloading AsynController::writeReadController() is necessary because simply 
// adapting outString_ and parsing inString_ doesn't work. AsynController::writeReadController()
// interprets outString_ starting with 0x00 as an empty string.
//
asynStatus StepperController::writeReadController()
{
  size_t nread;
  return writeReadController(outString_, inString_, sizeof(inString_), &nread, DEFAULT_CONTROLLER_TIMEOUT);
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus StepperController::writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout)
{
  size_t nwrite;
  asynStatus status;
  int eomReason;
  // const char *functionName="writeReadController";

  // Prepend 0x00 0x07 to output if not already present
  // Create a new buffer with prepended 0x00 0x07
  char *prependedOutput = (char *)malloc(strlen(output) + 3); // +2 for header, +1 for null terminator
  prependedOutput[0] = 0x00;
  prependedOutput[1] = 0x07;
  memcpy(prependedOutput + 2, output, strlen(output) + 1); // +1 for null terminator

  status = pasynOctetSyncIO->writeRead(pasynUserController_, prependedOutput,
                                       strlen(output) + 2, input, maxChars, timeout,
                                       &nwrite, nread, &eomReason);

  // Strip 0x00 0x07 from inString if present
  if (input[0] == 0x00 && input[1] == 0x07) {
    memmove(input, input + 2, strlen(input + 2) + 1);
    *nread -= 2;
  }

  return status;
}



/** Creates a new StepperController object.
  * \param[in] portName            The name of the asyn port that will be created for this driver
  * \param[in] StepperPortName     The name of the drvAsynSerialPort that was created previously to connect to the Stepper controller 
  * \param[in] numAxes             The number of axes that this controller supports 
  * \param[in] movingPollPeriod    The time between polls when any axis is moving 
  * \param[in] idlePollPeriod      The time between polls when no axis is moving 
  */
StepperController::StepperController(const char *portName, const char *StepperPortName, int numAxes, 
                                 double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_Stepper_PARAMS, 
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  StepperAxis *pAxis;
  static const char *functionName = "StepperController::StepperController";

  /* Connect to Stepper controller */
  status = pasynOctetSyncIO->connect(StepperPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s: cannot connect to Applied Motion Stepper controller\n",
      functionName);
  }
  for (axis=0; axis<numAxes; axis++) {
    pAxis = new StepperAxis(this, axis);
  }

  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new StepperController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] StepperPortName       The name of the drvAsynIPPPort that was created previously to connect to the Stepper controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int StepperCreateController(const char *portName, const char *StepperPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  StepperController *pStepperController
    = new StepperController(portName, StepperPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pStepperController = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void StepperController::report(FILE *fp, int level)
{
  fprintf(fp, "Stepper motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an StepperAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
StepperAxis* StepperController::getAxis(asynUser *pasynUser)
{
  return static_cast<StepperAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an StepperAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
StepperAxis* StepperController::getAxis(int axisNo)
{
  return static_cast<StepperAxis*>(asynMotorController::getAxis(axisNo));
}


// These are the StepperAxis methods

/** Creates a new StepperAxis object.
  * \param[in] pC Pointer to the StepperController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
StepperAxis::StepperAxis(StepperController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{  
  asynStatus status;

  // Set output format to decimal (not hexadecimal)
  sprintf(pC_->outString_, "IFD");
  status = pC_->writeReadController();

  // Initialize stepsPerRevolution
  sprintf(pC_->outString_, "EG");
  status = pC_->writeReadController();
  sscanf(pC_->inString_, "EG=%d", &stepsPerRevolution);

}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void StepperAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n",
            axisNo_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus StepperAxis::sendAccelAndVelocity(double acceleration, double velocity) 
{
  asynStatus status;
  // static const char *functionName = "Stepper::sendAccelAndVelocity";

  // Send the velocity
  // Velocity is in rev/sec
  sprintf(pC_->outString_, "VE%f", velocity);
  status = pC_->writeReadController();

  // Send the acceleration
  // acceleration is in rev/sec/sec
  sprintf(pC_->outString_, "AC%f", acceleration);
  status = pC_->writeReadController();
  return status;
}


asynStatus StepperAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "StepperAxis::move";

  status = sendAccelAndVelocity(acceleration, maxVelocity);
  
  // convert position to steps
  int steps = NINT(position * stepsPerRevolution);

  sprintf(pC_->outString_, "DI%d", steps);
  status = pC_->writeReadController();
  
  if (relative) {
    sprintf(pC_->outString_, "FL");
  } else {
    sprintf(pC_->outString_, "FP");
  }
  status = pC_->writeReadController();

  return status;
}

asynStatus StepperAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status;
  // static const char *functionName = "StepperAxis::home";

  status = asynDisabled;
  return status;
}


asynStatus StepperAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  static const char *functionName = "StepperAxis::moveVelocity";

  asynPrint(pasynUser_, ASYN_TRACE_FLOW,
    "%s: minVelocity=%f, maxVelocity=%f, acceleration=%f\n",
    functionName, minVelocity, maxVelocity, acceleration);
    
  status = sendAccelAndVelocity(acceleration, maxVelocity);

  // TODO: Start jogging

  status = asynDisabled;
  return status;
}

asynStatus StepperAxis::stop(double acceleration )
{
  asynStatus status;
  //static const char *functionName = "StepperAxis::stop";

  sprintf(pC_->outString_, "ST");

  status = pC_->writeReadController();
  return status;
}

asynStatus StepperAxis::setPosition(double position)
{
  asynStatus status;
  //static const char *functionName = "StepperAxis::setPosition";

  int steps = NINT(position * stepsPerRevolution);

  sprintf(pC_->outString_, "SP%d", steps);
  status = pC_->writeReadController();
  return status;
}


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status, 
  * and the drive power-on status. 
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus StepperAxis::poll(bool *moving)
{ 
  int done;
  int motorEnabled;
  int limit;
  double position;
  int statusCode;
  int alarmCode;
  asynStatus comStatus;

  // Read the current motor position
  // IP - Immediate Position 
  sprintf(pC_->outString_, "IP");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "IP=100", which gives the position in 
  // steps. Convert to revs. 
  int position_steps;
  sscanf(pC_->inString_, "IP=%d", &position_steps);
  position = static_cast<double>(position_steps) / stepsPerRevolution;
  setDoubleParam(pC_->motorPosition_, position);

  // Read the moving status of this motor
  // SC - Status Code

  // Responds with a four-character hex code
  sprintf(pC_->outString_, "SC");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "SC=0000", with a four-character hex code.
  sscanf(pC_->inString_, "SC=%x", &statusCode);
  // 0x0010 gives Motor is Moving status.
  done = !(statusCode & 0x0010) ? 1:0;
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;

  // Read the limit status
  // AL - Alarm Code

  sprintf(pC_->outString_, "AL");
  comStatus = pC_->writeReadController();
  if (comStatus) goto skip;
  // The response string is of the form "AL=0000", with a four-character hex code
  // of which 0001 indicates a position limit, 0002 a CCW limit, and 0004 a CW
  // limit.

  // Convert the 4-character hex code after "AL=" to integer
  sscanf(pC_->inString_, "AL=%x", &alarmCode);

  limit = (alarmCode & 0x0004) ? 1:0;
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (alarmCode & 0x0002) ? 1:0;
  setIntegerParam(pC_->motorStatusLowLimit_, limit);

  // Get the drive power on status from "SC" command output
  // 0x0001 gives Motor Enabled and in position status
  motorEnabled = (statusCode & 0x0001) ? 1:0;
  setIntegerParam(pC_->motorStatusPowerOn_, motorEnabled);
  setIntegerParam(pC_->motorStatusProblem_, 0);

  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg StepperCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg StepperCreateControllerArg1 = {"Stepper port name", iocshArgString};
static const iocshArg StepperCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg StepperCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg StepperCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const StepperCreateControllerArgs[] = {&StepperCreateControllerArg0,
                                                             &StepperCreateControllerArg1,
                                                             &StepperCreateControllerArg2,
                                                             &StepperCreateControllerArg3,
                                                             &StepperCreateControllerArg4};
static const iocshFuncDef StepperCreateControllerDef = {"StepperCreateController", 5, StepperCreateControllerArgs};
static void StepperCreateContollerCallFunc(const iocshArgBuf *args)
{
  StepperCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void StepperRegister(void)
{
  iocshRegister(&StepperCreateControllerDef, StepperCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(StepperRegister);
}
