/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <iostream>
#include "LimeLight/ControlMode.h"
#include "NetworkTables/NetworkTable.h"
class LimeLight {
 public:
  LimeLight();
  void FindTarget();
  void StopTarget();
};
