/*
FILENAME...   StepperDriver.h
USAGE...      Motor driver support for the Applied Motion Stepper controllers
              Includes all ST5/10, STF, STM, SWM, STAC5 and STAC6 drives

Adapted from ACS MCB-4B driver by Mark Rivers

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_Stepper_AXES 4

// No controller-specific parameters yet
#define NUM_Stepper_PARAMS 0  

class epicsShareClass StepperAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  StepperAxis(class StepperController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);

private:
  StepperController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  
  int stepsPerRevolution;  /* pulses per revolution for electronic gearing */
  asynStatus sendAccelAndVelocity(double accel, double velocity);
  
friend class StepperController;
};

class epicsShareClass StepperController : public asynMotorController {
public:
  StepperController(const char *portName, const char *StepperPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  StepperAxis* getAxis(asynUser *pasynUser);
  StepperAxis* getAxis(int axisNo);

  asynStatus writeReadController();
  asynStatus writeReadController(const char *output, char *input, 
                                                    size_t maxChars, size_t *nread, double timeout);

friend class StepperAxis;
};
