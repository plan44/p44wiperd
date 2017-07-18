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

#ifndef __p44wiperd__dcmotordriver__
#define __p44wiperd__dcmotordriver__

#include "p44utils_common.hpp"

#include "serialcomm.hpp"
#include "digitalio.hpp"
#include "analogio.hpp"

using namespace std;

namespace p44 {


  class DcMotorDriver;


  typedef boost::function<void (double aCurrentPower, int aDirection, ErrorPtr aError)> DCMotorStatusCB;


  typedef boost::intrusive_ptr<DcMotorDriver> DcMotorDriverPtr;
  class DcMotorDriver : public P44Obj
  {
    typedef P44Obj inherited;

    AnalogIoPtr pwmOutput;
    DigitalIoPtr cwDirectionOutput;
    DigitalIoPtr ccwDirectionOutput;

    int currentDirection;
    double currentPower;

    long rampTicket;

  public:

    /// Create a motor controller
    /// @param aPWMOutput a 0..100 Analog output controlling the PWM signal for the DC motor
    /// @param aCWDirectionOutput a digital output enabling clockwise motor operation.
    ///   If no CCW output is set, this is assumed to just switch the direction (1=CW, 0=CCW)
    ///   If no CW output is set, this is assumed to be a unidirectional motor which is only controlled via the PWM
    /// @param aCCWDirectionOutput a digital output enabling counter clockwise motor operation.
    ///   If this is set, CW and CCW are assumed to each control one of the half bridges,
    ///   so using CCW==CW will drive the motor, and CCW==CW will brake it
    DcMotorDriver(const char *aPWMOutput, const char *aCWDirectionOutput = NULL, const char *aCCWDirectionOutput = NULL);
    virtual ~DcMotorDriver();

    /// ramp motor from current power to another power
    /// @param aPower 0..100 new brake or drive power to apply
    /// @param aDirection driving direction: 1 = CW, -1 = CCW, 0 = hold/brake
    /// @param aFullRampTime number of seconds for a full scale (0..100 or vice versa) power change.
    ///   Note that ramping from one aDirection to another will execute two separate ramps in sequence
    /// @param aRampExp ramp exponent (1=linear, 2=quadratic, 3=cubic...
    /// @param aRampDoneCB will be called at end of ramp
    void rampToPower(double aPower, int aDirection, double aFullRampTime, double aRampExp, DCMotorStatusCB aRampDoneCB = NULL);

    /// stop immediately, no braking
    void stop();


  protected:


  private:

    void setPower(double aPower, int aDirection);
    void setDirection(int aDirection);
    void rampStep(double aTargetPower, double aPowerStep, MLMicroSeconds aRemainingTime, double aRampExp, DCMotorStatusCB aRampDoneCB);

  };



} // namespace p44

#endif /* defined(__p44wiperd__dcmotordriver__) */
