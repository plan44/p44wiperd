//
//  Copyright (c) 2017 plan44.ch / Lukas Zeller, Zurich, Switzerland
//
//  Author: Lukas Zeller <luz@plan44.ch>
//
//  This file is part of p44wiperd.
//
//  p44ayabd is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  p44ayabd is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with p44ayabd. If not, see <http://www.gnu.org/licenses/>.
//

#include "dcmotordriver.hpp"

#include "consolekey.hpp"
#include "application.hpp"

#include <math.h>

using namespace p44;


#pragma mark - DCMotorDriver

DcMotorDriver::DcMotorDriver(const char *aPWMOutput, const char *aCWDirectionOutput, const char *aCCWDirectionOutput) :
  currentPower(0),
  currentDirection(0),
  sequenceTicket(0)
{
  pwmOutput = AnalogIoPtr(new AnalogIo(aPWMOutput, true, 0)); // off to begin with
  if (aCWDirectionOutput) {
    cwDirectionOutput = DigitalIoPtr(new DigitalIo(aCWDirectionOutput, true, false));
    if (aCCWDirectionOutput) {
      ccwDirectionOutput = DigitalIoPtr(new DigitalIo(aCCWDirectionOutput, true, false));
    }
  }
  setPower(0, 0);
}


DcMotorDriver::~DcMotorDriver()
{
  // stop power to motor
  setPower(0, 0);
}



void DcMotorDriver::setDirection(int aDirection)
{
  if (cwDirectionOutput) {
    cwDirectionOutput->set(aDirection>0);
    if (ccwDirectionOutput) {
      ccwDirectionOutput->set(aDirection<0);
    }
  }
  if (aDirection!=currentDirection) {
    LOG(LOG_DEBUG, "Direction changed to %d", aDirection);
    currentDirection = aDirection;
  }
}



void DcMotorDriver::setPower(double aPower, int aDirection)
{
  if (aPower<=0) {
    // no power
    // - disable PWM
    pwmOutput->setValue(0);
    // - off (= hold/brake with no power)
    setDirection(0);
  }
  else {
    // determine current direction
    if (currentDirection!=0 && aDirection!=0 && aDirection!=currentDirection) {
      // avoid reversing direction with power on
      pwmOutput->setValue(0);
      setDirection(0);
    }
    // now set desired direction and power
    setDirection(aDirection);
    pwmOutput->setValue(aPower);
  }
  if (aPower!=currentPower) {
    LOG(LOG_DEBUG, "Power changed to %.2f%%", aPower);
    currentPower = aPower;
  }
}


#define RAMP_STEP_TIME (20*MilliSecond)


void DcMotorDriver::stop()
{
  stopSequences();
  setPower(0, 0);
}


void DcMotorDriver::stopSequences()
{
  MainLoop::currentMainLoop().cancelExecutionTicket(sequenceTicket);
}



static double powerToOut(double aPower, double aExp)
{
  if (aExp==0) return aPower;
  return 100*((exp(aPower*aExp/100)-1)/(exp(aExp)-1));
}




void DcMotorDriver::rampToPower(double aPower, int aDirection, double aFullRampTime, double aRampExp, DCMotorStatusCB aRampDoneCB)
{
  LOG(LOG_DEBUG, "+++ new ramp: power: %.2f%%..%.2f%%, direction:%d..%d with full ramp time %.3f Seconds", currentPower, aPower, currentDirection, aDirection, aFullRampTime);
  MainLoop::currentMainLoop().cancelExecutionTicket(sequenceTicket);
  if (aDirection!=currentDirection) {
    if (currentPower!=0) {
      // ramp to zero first, then ramp up to new direction
      LOG(LOG_DEBUG, "Ramp trough different direction modes -> first ramp power down, then up again");
      rampToPower(0, currentDirection, aFullRampTime, aRampExp, boost::bind(&DcMotorDriver::rampToPower, this, aPower, aDirection, aFullRampTime, aRampExp, aRampDoneCB));
      return;
    }
    // set new direction
    setDirection(aDirection);
  }
  // limit
  if (aPower>100) aPower=100;
  else if (aPower<0) aPower=0;
  // ramp to new value
  double rampRange = aPower-currentPower;
  MLMicroSeconds totalRampTime = fabs(rampRange)/100*aFullRampTime*Second;
  double powerStep = rampRange;
  if (totalRampTime>0) {
    powerStep = rampRange*RAMP_STEP_TIME/totalRampTime;
  }
  LOG(LOG_DEBUG, "Ramp power from %.2f%% to %.2f%% in %.3f Seconds, power step %.3f", currentPower, aPower, (double)totalRampTime/Second, powerStep);
  // now execute the ramp
  rampStep(aPower, powerStep, totalRampTime, aRampExp, aRampDoneCB);
}



void DcMotorDriver::rampStep(double aTargetPower, double aPowerStep, MLMicroSeconds aRemainingTime, double aRampExp, DCMotorStatusCB aRampDoneCB)
{
  LOG(LOG_DEBUG, "ramp step, remaining time= %lld uS", aRemainingTime);
  if (aRemainingTime<RAMP_STEP_TIME) {
    // finalize
    setPower(powerToOut(aTargetPower, aRampExp), currentDirection);
    LOG(LOG_DEBUG, "--- end of ramp");
    // call back
    if (aRampDoneCB) aRampDoneCB(currentPower, currentDirection, ErrorPtr());
  }
  else {
    // set power for this step
    setPower(powerToOut(currentPower+aPowerStep, aRampExp), currentDirection);
    // schedule next step
    sequenceTicket = MainLoop::currentMainLoop().executeOnce(boost::bind(&DcMotorDriver::rampStep, this, aTargetPower, aPowerStep, aRemainingTime-RAMP_STEP_TIME, aRampExp, aRampDoneCB), RAMP_STEP_TIME);
  }
}


void DcMotorDriver::runSequence(SequenceStepList aSteps, DCMotorStatusCB aSequenceDoneCB)
{
  stopSequences();
  if (aSteps.size()==0) {
    // done
    if (aSequenceDoneCB) aSequenceDoneCB(currentPower, currentDirection, ErrorPtr());
  }
  // next step
  SequenceStep step = aSteps.front();
  rampToPower(step.power, step.direction, step.rampTime, step.rampExp, boost::bind(&DcMotorDriver::sequenceStepDone, this, aSteps, aSequenceDoneCB, _3));
}


void DcMotorDriver::sequenceStepDone(SequenceStepList aSteps, DCMotorStatusCB aSequenceDoneCB, ErrorPtr aError)
{
  if (!Error::isOK(aError)) {
    // error, abort sequence
    if (aSequenceDoneCB) aSequenceDoneCB(currentPower, currentDirection, aError);
    return;
  }
  // launch next step after given run time
  SequenceStep step = aSteps.front();
  aSteps.pop_front();
  MainLoop::currentMainLoop().executeTicketOnce(sequenceTicket, boost::bind(&DcMotorDriver::runSequence, this, aSteps, aSequenceDoneCB), step.runTime*Second);
}


//void DcMotorDriver::runConstSequence(const SequenceStep aSteps[], DCMotorStatusCB aSequenceDoneCB)
//{
//  SequenceStepList steps;
//  while (aSteps && aSteps->power>0) {
//    steps.push_back(*aSteps);
//    aSteps++;
//  }
//  runSequence(steps, aSequenceDoneCB);
//}






