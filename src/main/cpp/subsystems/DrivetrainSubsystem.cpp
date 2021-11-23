/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSubsystem.h"
#include <iostream>
#define talonsrx std::make_unique<TalonSRX>

DrivetrainSubsystem::DrivetrainSubsystem() : Subsystem("DrivetrainSubsystem")
{
  // name = <speedcontroller>(motorcontrollerID, motortype)
  // m_leftMotorA = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_leftMotorA, rev::CANSparkMax::MotorType::kBrushless);
  // m_leftMotorB = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_leftMotorB, rev::CANSparkMax::MotorType::kBrushless);
  // m_leftMotorC = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_leftMotorC, rev::CANSparkMax::MotorType::kBrushless);

  // m_rightMotorA = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_rightMotorA, rev::CANSparkMax::MotorType::kBrushless);
  // m_rightMotorB = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_rightMotorB, rev::CANSparkMax::MotorType::kBrushless);
  // m_rightMotorC = std::make_unique<rev::CANSparkMax>(motorcontrollerID::k_rightMotorC, rev::CANSparkMax::MotorType::kBrushless);

  //set idlemode
  // m_leftMotorA->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_leftMotorB->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_leftMotorC->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightMotorA->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightMotorB->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  // m_rightMotorC->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  // m_leftMotorA->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  // m_leftMotorB->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  // m_leftMotorC->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  // m_rightMotorA->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  // m_rightMotorB->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  // m_rightMotorC->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  //invert direction
  // m_leftMotorA->SetInverted(false);
  // m_rightMotorA->SetInverted(true);

  //set followers
  // m_leftMotorB->Follow(*m_leftMotorA);
  // m_leftMotorC->Follow(*m_leftMotorA);
  // m_rightMotorB->Follow(*m_rightMotorA);
  // m_rightMotorC->Follow(*m_rightMotorA);

  //set encoder
  // m_leftEncoder = std::make_unique<rev::CANEncoder>(*m_leftMotorA, rev::CANEncoder::EncoderType::kHallSensor, 42);
  // m_rightEncoder = std::make_unique<rev::CANEncoder>(*m_rightMotorA, rev::CANEncoder::EncoderType::kHallSensor, 42);

  //Set the position of the encoder.
  // m_leftEncoder->SetPosition(0);
  // m_rightEncoder->SetPosition(0);

  m_leftMotorA = make<TalonSRX>(motorcontrollerID::k_leftMotorA);
  m_leftMotorB = make<TalonSRX>(motorcontrollerID::k_leftMotorB);
  m_leftMotorC = make<TalonSRX>(motorcontrollerID::k_leftMotorC);

  m_rightMotorA = make<TalonSRX>(motorcontrollerID::k_rightMotorA);
  m_rightMotorB = make<TalonSRX>(motorcontrollerID::k_rightMotorB);
  m_rightMotorC = make<TalonSRX>(motorcontrollerID::k_rightMotorC);

  m_leftMotorB->Follow(*m_leftMotorA);
  m_leftMotorC->Follow(*m_leftMotorA);
  m_rightMotorB->Follow(*m_rightMotorA);
  m_rightMotorC->Follow(*m_rightMotorA);

  //invert direction
  m_leftMotorA->SetInverted(InvertType::None);
  m_rightMotorA->SetInverted(InvertType::InvertMotorOutput);

  m_leftMotorB->SetInverted(InvertType::None);
  m_rightMotorB->SetInverted(InvertType::InvertMotorOutput);

  m_leftMotorC->SetInverted(InvertType::None);
  m_rightMotorC->SetInverted(InvertType::InvertMotorOutput);

  //m_leftMotorA->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  //m_rightMotorA->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  m_leftMotorA->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
  m_rightMotorA->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 10);
  
  m_leftMotorA->SetSelectedSensorPosition(0, 0,10);
  m_rightMotorA->SetSelectedSensorPosition(0, 0,10);

  //velocity settings
  m_leftMotorA->Config_kP(0, 1.73);
  m_leftMotorA->Config_kI(0, 0);
  m_leftMotorA->Config_kD(0, 0);

  m_rightMotorA->Config_kP(0, 1.73);
  m_rightMotorA->Config_kI(0, 0);
  m_rightMotorA->Config_kD(0, 0);

  //rotation settings
  m_leftMotorA->Config_kP(1, 20);
  m_leftMotorA->Config_kI(1, 0);
  m_leftMotorA->Config_kD(1, 0);

  m_rightMotorA->Config_kP(1, 20);
  m_rightMotorA->Config_kI(1, 0);
  m_rightMotorA->Config_kD(1, 0);

  //PID slot 0 is the default
  m_leftMotorA->SelectProfileSlot(0, 0);
  m_rightMotorA->SelectProfileSlot(0, 0);

  m_leftMotorA->SetSensorPhase(false);
  m_rightMotorA->SetSensorPhase(false);

  //gyro
  try
  {
    m_ahrs = make<AHRS>(SPI::Port::kMXP);
  }
  catch (std::exception ex)
  {
    std::string err_string = "Error initalizing navX-MXP";
    err_string += ex.what();
    DriverStation::ReportError(err_string.c_str());
  }
  m_ahrs->Reset();
}

void DrivetrainSubsystem::InitDefaultCommand()
{
  // Set the default command for a subsystem here.
  SetDefaultCommand(new CmdToAngle(1));
}
void DrivetrainSubsystem::SetLeftPower(double speed)
{
  m_leftMotorA->Set(ControlMode::PercentOutput, speed);
}
void DrivetrainSubsystem::SetRightPower(double speed)
{
  m_rightMotorA->Set(ControlMode::PercentOutput, speed);
}
inline float deadband(float joyValue)
{
  return (std::abs(joyValue) > 0.15f) ? joyValue : 0.0f;
}
void DrivetrainSubsystem::ArcadeDrive(frc::XboxController *joystick)
{
  double magnitude = deadband(joystick->GetRawAxis(1));//GetY(frc::GenericHID::JoystickHand::kLeftHand));
  double rotation = deadband(joystick->GetRawAxis(2));//GetX(frc::GenericHID::JoystickHand::kRightHand));
  //std::cout << Robot::m_drivetrain.GetLeftVelocity() << std::endl;
  frc::SmartDashboard::PutNumber("magnitude value: ", magnitude);
  frc::SmartDashboard::PutNumber("rotation value: ", rotation); 
  //m_leftMotorA->Set(ControlMode::PercentOutput, (magnitude + rotation));
  //m_rightMotorA->Set(ControlMode::PercentOutput, (magnitude - rotation));
  m_leftMotorA->Set(ControlMode::Velocity, (magnitude + rotation));
  m_rightMotorA->Set(ControlMode::Velocity, (magnitude - rotation));//300
}

void DrivetrainSubsystem::ToSpeedLeft(double speed)
{
  m_leftMotorA->Set(ControlMode::Velocity, speed * 497 * 4);
}

void DrivetrainSubsystem::ToSpeedRight(double speed)
{
  m_rightMotorA->Set(ControlMode::Velocity, speed * 497 * 4);
}

void DrivetrainSubsystem::ToRotationLeft(double rotation)
{
  m_leftMotorA->Set(ControlMode::Position, (rotation) * 497 * 4);
}

void DrivetrainSubsystem::ToRotationRight(double rotation)
{
  m_rightMotorA->Set(ControlMode::Position, (rotation) * 497 * 4);
}

void DrivetrainSubsystem::NeutralMode(bool isEnable)
{
  if (isEnable)
  {
    m_leftMotorA->SetNeutralMode(NeutralMode::Brake);
    m_rightMotorA->SetNeutralMode(NeutralMode::Brake);

    m_leftMotorB->SetNeutralMode(NeutralMode::Brake);
    m_rightMotorB->SetNeutralMode(NeutralMode::Brake);

    m_leftMotorC->SetNeutralMode(NeutralMode::Brake);
    m_rightMotorC->SetNeutralMode(NeutralMode::Brake);
  }
  else
  {
    m_leftMotorA->SetNeutralMode(NeutralMode::Coast);
    m_rightMotorA->SetNeutralMode(NeutralMode::Coast);

    m_leftMotorB->SetNeutralMode(NeutralMode::Coast);
    m_rightMotorB->SetNeutralMode(NeutralMode::Coast);

    m_leftMotorC->SetNeutralMode(NeutralMode::Coast);
    m_rightMotorC->SetNeutralMode(NeutralMode::Coast);
  }
}
double DrivetrainSubsystem::GetLeftEncoder()
{
  //return m_leftEncoder->GetPosition();
  return m_leftMotorA->GetSelectedSensorPosition();
}
double DrivetrainSubsystem::GetRightEncoder()
{
  //return m_rightEncoder->GetPosition();
  return m_rightMotorA->GetSelectedSensorPosition();
}

double DrivetrainSubsystem::GetGyroAngle()
{
  return m_ahrs->GetAngle();
}

double DrivetrainSubsystem::GetLeftVelocity()
{
  return m_leftMotorA->GetSelectedSensorVelocity();
}

double DrivetrainSubsystem::GetRightVelocity()
{
  return m_rightMotorA->GetSelectedSensorVelocity();
}
