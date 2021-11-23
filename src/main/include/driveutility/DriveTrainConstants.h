/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#define make std::make_unique
#define shared std::shared_ptr

namespace motorcontrollerID
{
constexpr int
    k_leftMotorA = 1,
    k_leftMotorB = 3,
    k_leftMotorC = 5,

    k_rightMotorA = 2,
    k_rightMotorB = 4,
    k_rightMotorC = 6;
}
class DriveTrainConstants {
 public:
  DriveTrainConstants();
};
