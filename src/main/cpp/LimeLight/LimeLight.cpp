/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "LimeLight/LimeLight.h"

LimeLight::LimeLight() {

}
void LimeLight::FindTarget(){

}
void LimeLight::StopTarget(){
//std::cout << "Sonic Value: " << distanceSensor->GetValue() << std::endl;
    // Get limelight table for reading tracking data
    std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
    table->PutNumber("pipeline", 0); //Sets limelight’s current pipeline
    //turn the lights off
    table->PutNumber("ledMode", ControlMode::LedMode::kforceOff); //force off
    table->PutNumber("camMode", ControlMode::CamMode::kvisionprocessingOff); //Driver Camera (Increases exposure, disables vision processing)
}