/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DrivePercent.h"
#include "Robot.h"

DrivePercent::DrivePercent() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&globalRobot.driveSystem);
}

// Called just before this Command runs the first time
void DrivePercent::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DrivePercent::Execute() {globalRobot.driveSystem.JoystickPercentDrive(globalRobot.oi.xbox->GetRawAxis(0), globalRobot.oi.xbox->GetRawAxis(1));}

// Make this return true when this Command no longer needs to run execute()
bool DrivePercent::IsFinished() { return false; }

// Called once after isFinished returns true
void DrivePercent::End() {globalRobot.driveSystem.JoystickPercentDrive(0, 0);}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DrivePercent::Interrupted() {End();}
