/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Drive/CmdToAngle.h"
#include <iostream>
#include "Robot.h"
#include "LimeLight/LimeLight.h"
CmdToAngle::CmdToAngle(double angle) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
  this->angle = angle;
}

// Called just before this Command runs the first time
void CmdToAngle::Initialize() {}
// Called repeatedly when this Command is scheduled to run
void CmdToAngle::Execute() {
  LimeLight limeLight;
  // double error = Robot::m_drivetrain.GetGyroAngle() - angle;
  // std::cout << error << std::endl;

  // double outputRight = error * kP;
  // Robot::m_drivetrain.ToSpeedLeft(-outputRight);
  // Robot::m_drivetrain.ToSpeedRight(outputRight);
  limeLight.StopTarget();
}

// Make this return true when this Command no longer needs to run execute()
bool CmdToAngle::IsFinished() { return false; }

// Called once after isFinished returns true
void CmdToAngle::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdToAngle::Interrupted() {}
