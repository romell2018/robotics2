/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "commands/Drive/TempPIDRotation.h"
#include "commands/Drive/CmdArcadeDrive.h"
#include "commands/Drive/TestAuto.h"
#include "commands/Drive/CmdSetLeftSpeed.h"
#include "commands/Drive/CmdSetRightSpeed.h"
#include "commands/Drive/CmdToAngle.h"
#include "frc/XboxController.h"

OI::OI()
{
  // Process operator interface input here.
  joy = new frc::XboxController(0);

  arcadeDrive = new frc::JoystickButton(joy, 1);
  arcadeDrive->WhenPressed(new CmdArcadeDrive());

  //toRotation = new frc::JoystickButton(joy, 2);
  ///toRotation->WhenPressed(new TestAuto());

  //toSpeedLow = new frc::JoystickButton(joy, 3);
  //toSpeedLow->WhenPressed(new CmdToAngle(90));

  toSpeedHigh = new frc::JoystickButton(joy, 4);
  toSpeedHigh->WhenPressed(new CmdToAngle(0));

}
frc::XboxController *OI::getJoy()
{
    return joy;
}
