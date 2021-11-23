/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Drive/CmdArcadeDrive.h"
#include "Robot.h"

#include <iostream>

 CmdArcadeDrive::CmdArcadeDrive()
{
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_drivetrain);
}

// Called just before this Command runs the first time
void CmdArcadeDrive::Initialize() {
  Robot::m_drivetrain.m_leftMotorA->SetSensorPhase(false);
  Robot::m_drivetrain.m_rightMotorA->SetSensorPhase(false);
  Robot::m_drivetrain.m_leftMotorA->SelectProfileSlot(0, 0);
  Robot::m_drivetrain.m_rightMotorA->SelectProfileSlot(0, 0);
}

// Called repeatedly when this Command is scheduled to run
void CmdArcadeDrive::Execute()
{
  
 //Robot::m_drivetrain.ArcadeDrive(Robot::m_oi.getJoy());
  Robot::m_drivetrain.ArcadeDrive(Robot::m_oi.getJoy());

  std::cout << Robot::m_drivetrain.GetLeftVelocity() << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool CmdArcadeDrive::IsFinished() { return false; }

// Called once after isFinished returns true
void CmdArcadeDrive::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CmdArcadeDrive::Interrupted() {}
