/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "commands/Drive/TempPIDRotation.h"
#include <iostream>
TempPIDRotation::TempPIDRotation(double left, double right) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::m_drivetrain);
  this->left = left;
  this->right = right;
}

// Called just before this Command runs the first time
void TempPIDRotation::Initialize() {
  Robot::m_drivetrain.m_leftMotorA->SetSensorPhase(true);
  Robot::m_drivetrain.m_rightMotorA->SetSensorPhase(true);
  Robot::m_drivetrain.m_leftMotorA->SelectProfileSlot(1, 0);
  Robot::m_drivetrain.m_rightMotorA->SelectProfileSlot(1, 0);
}

// Called repeatedly when this Command is scheduled to run
void TempPIDRotation::Execute() {

  //double left = Robot::m_oi.getJoy()->GetY(frc::XboxController::JoystickHand::kLeftHand);
  //double right = Robot::m_oi.getJoy()->GetY(frc::XboxController::JoystickHand::kRightHand);
  Robot::m_drivetrain.ToRotationLeft(this->left);
  Robot::m_drivetrain.ToRotationRight(this->right);
  std::cout << this->left / (497 * 4) << ',' << this->right / (497 * 4) << std::endl;
  std::cout << Robot::m_drivetrain.GetLeftEncoder() / (497 * 4) << ',' << Robot::m_drivetrain.GetRightEncoder() / (497 * 4) << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
//allowable error 10
//test if it passes the target
bool TempPIDRotation::IsFinished() { 
  //static int pastLeft = Robot::m_drivetrain.GetLeftEncoder() - left;
  //static int pastRight = Robot::m_drivetrain.GetRightEncoder() - right;
  return 0.1 > abs(Robot::m_drivetrain.GetLeftEncoder() / (497 * 4) - left) && 0.1 > abs(Robot::m_drivetrain.GetRightEncoder() / (497 * 4) - right);
}

// Called once after isFinished returns true
void TempPIDRotation::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TempPIDRotation::Interrupted() {}
