/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "rev/CANSparkMax.h"
#include "rev/CANEncoder.h"
#include "frc/WPILib.h"
#include "driveutility/DriveTrainConstants.h"
#include "AHRS.h"
#include "ctre/Phoenix.h"
#include "frc/smartdashboard/SmartDashboard.h"

//commands
#include "commands/Drive/CmdArcadeDrive.h"
#include "commands/Drive/TempPIDRotation.h"
#include "commands/Drive/CmdToAngle.h"
class DrivetrainSubsystem : public frc::Subsystem
{
public:
  DrivetrainSubsystem();
  void InitDefaultCommand() override;
  void SetLeftPower(double speed);
  void SetRightPower(double speed);
  void ArcadeDrive(frc::XboxController *joystick);

  void ToSpeedLeft(double speed);
  void ToSpeedRight(double speed);
  void ToRotationLeft(double rotation);
  void ToRotationRight(double rotation);

  void NeutralMode(bool isEnable);
  double GetLeftEncoder();
  double GetRightEncoder();
  double GetGyroAngle();
  double GetLeftVelocity();
  double GetRightVelocity();
  //motorcontroller
  // std::shared_ptr<rev::CANSparkMax> m_leftMotorA;
  // std::shared_ptr<rev::CANSparkMax> m_leftMotorB;
  // std::shared_ptr<rev::CANSparkMax> m_leftMotorC;

  // std::shared_ptr<rev::CANSparkMax> m_rightMotorA;
  // std::shared_ptr<rev::CANSparkMax> m_rightMotorB;
  // std::shared_ptr<rev::CANSparkMax> m_rightMotorC;

  shared<TalonSRX> m_leftMotorA;
  shared<TalonSRX> m_leftMotorB;
  shared<TalonSRX> m_leftMotorC;

  shared<TalonSRX> m_rightMotorA;
  shared<TalonSRX> m_rightMotorB;
  shared<TalonSRX> m_rightMotorC;

  //motor encoder
  // std::shared_ptr<rev::CANEncoder> m_leftEncoder;
  // std::shared_ptr<rev::CANEncoder> m_rightEncoder;
  //gyro
  shared<AHRS> m_ahrs;
};
// TODO: vision limelight and usbcam
// // periodically read voltage, temperature, and applied output and publish to SmartDashboard
// frc::SmartDashboard::PutNumber("Voltage", m_motor.GetBusVoltage());
// frc::SmartDashboard::PutNumber("Temperature", m_motor.GetMotorTemperature());
// frc::SmartDashboard::PutNumber("Output", m_motor.GetAppliedOutput());
