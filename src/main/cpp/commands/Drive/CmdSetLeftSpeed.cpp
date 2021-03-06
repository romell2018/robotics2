/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "commands/Drive/CmdSetLeftSpeed.h"

CmdSetLeftSpeed::CmdSetLeftSpeed(double speed) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
  this->speed = speed;
}

// Called just before this Command runs the first time
void CmdSetLeftSpeed::Initialize() {
  Robot::m_drivetrain.m_leftMotorA->SelectProfileSlot(0, 0);
}

// Called repeatedly when this Command is scheduled to run
void CmdSetLeftSpeed::Execute() {
  Robot::m_drivetrain.ToSpeedLeft(speed);
}

// Make this return true when this Command no longer needs to run execute()
bool CmdSetLeftSpeed::IsFinished() { return false; }

// Called once after isFinished returns true
void CmdSetLeftSpeed::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdSetLeftSpeed::Interrupted() {}
