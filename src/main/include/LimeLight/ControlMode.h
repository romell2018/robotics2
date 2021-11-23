/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class ControlMode
{
public:
  ControlMode();
  enum LedMode
  {
    kforceOn = 0,
    kforceOff = 1,
  };
  enum CamMode {
    kvisionprocessingOn = 0,
    kvisionprocessingOff = 1,
  };
  enum PipelineMode {
  };
private:
};