/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Drive/CmdSetRightSpeed.h"
#include "Robot.h"

CmdSetRightSpeed::CmdSetRightSpeed(double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
  this->speed = speed;
}

// Called just before this Command runs the first time
void CmdSetRightSpeed::Initialize() {
  Robot::m_drivetrain.m_rightMotorA->SelectProfileSlot(0, 0);
}

// Called repeatedly when this Command is scheduled to run
void CmdSetRightSpeed::Execute() {
  Robot::m_drivetrain.ToSpeedRight(speed);
}

// Make this return true when this Command no longer needs to run execute()
bool CmdSetRightSpeed::IsFinished() { return false; }

// Called once after isFinished returns true
void CmdSetRightSpeed::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdSetRightSpeed::Interrupted() {}
