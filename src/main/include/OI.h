/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>


class OI
{
public:
  OI();
  frc::XboxController *getJoy();
private:
  frc::XboxController *joy;
  frc::JoystickButton *arcadeDrive;
  frc::JoystickButton *toRotation;
  frc::JoystickButton* toSpeedLow;
  frc::JoystickButton* toSpeedHigh;
   frc::XboxController *joy;
  frc::JoystickButton *arcadeDrive;
  frc::JoystickButton *toRotation;
  frc::JoystickButton* toSpeedLow;
  frc::JoystickButton* toSpeedHigh;
};
