/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DriveSystem.h"
#include "commands/DrivePercent.h"
#include "commands/DriveVelocity.h"

DriveSystem::DriveSystem() : Subsystem("DriveSystem"), frontLeftMotor(4), rearLeftMotor(10), frontRightMotor(1), rearRightMotor(2) 
{
  frontLeftMotor.SetInverted(false);
  rearLeftMotor.SetInverted(false);
  frontRightMotor.SetInverted(true);
  rearRightMotor.SetInverted(true);

  rearLeftMotor.SetSensorPhase(false);
  rearRightMotor.SetSensorPhase(true);

  rearLeftMotor.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);
  rearRightMotor.ConfigSelectedFeedbackSensor(QuadEncoder, 0, 0);

  rearLeftMotor.SetSelectedSensorPosition(0, 0, 0);
  rearRightMotor.SetSelectedSensorPosition(0, 0, 0);

  rearLeftMotor.ConfigClosedloopRamp(0, 0);
  rearRightMotor.ConfigClosedloopRamp(0, 0);

  frontLeftMotor.ConfigClosedloopRamp(0, 0);
  frontRightMotor.ConfigClosedloopRamp(0, 0);

  float kf = 0.17;
  float kp = 0.1;
  float ki = 0;
  float kd = 0;

  rearLeftMotor.Config_kF(0, kf, 0);
  rearLeftMotor.Config_kP(0, kp, 0);
  rearLeftMotor.Config_kI(0, ki, 0);
  rearLeftMotor.Config_kD(0, kd, 0);

  rearRightMotor.Config_kF(0, kf, 0);
  rearRightMotor.Config_kP(0, kp, 0);
  rearRightMotor.Config_kI(0, ki, 0);
  rearRightMotor.Config_kD(0, kd, 0);

  frontLeftMotor.Config_kF(0, kf, 0);
  frontLeftMotor.Config_kP(0, kp, 0);
  frontLeftMotor.Config_kI(0, ki, 0);
  frontLeftMotor.Config_kD(0, kd, 0);

  frontRightMotor.Config_kF(0, kf, 0);
  frontRightMotor.Config_kP(0, kp, 0);
  frontRightMotor.Config_kI(0, ki, 0);
  frontRightMotor.Config_kD(0, kd, 0);
}

void DriveSystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new DrivePercent());
  // SetDefaultCommand(new DriveVelocity());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveSystem::JoystickVelocityDrive(double x, double y)
{
  double l;
  double r;
  double leftOutput;
  double rightOutput;
  double increment = 2600;
  double maxVelocity = 3000;

  if (y > 0.2)
  {
    y = (y - 0.2) * (1 / .8) * maxVelocity;
  }
  else if (y < -0.2)
  {
    y = (y + 0.2) * 1 / .8 * maxVelocity;
  }
  else
  {
    y = 0;
  }

  if (x > 0.2)
  {
    x = (x - 0.2) * 1 / .8 * maxVelocity/2;
  }
  else if (x < -0.2)
  {
    x = (x + 0.2) * 1 / .8 * maxVelocity/2;
  }
  else
  {
    x = 0;
  }

  l = -y + x;
  r = -y - x;

  if (l > rearLeftMotor.GetSelectedSensorVelocity(0) + increment)
  {
    leftOutput = rearLeftMotor.GetSelectedSensorVelocity(0) + increment;
  }
  else if (l < rearLeftMotor.GetSelectedSensorVelocity(0) - increment)
  {
    leftOutput = rearLeftMotor.GetSelectedSensorVelocity(0) - increment;
  }
  else
  {
    leftOutput = l;
  }

  if (r > rearRightMotor.GetSelectedSensorVelocity(0) + increment)
  {
    rightOutput = rearRightMotor.GetSelectedSensorVelocity(0) + increment;
  }
  else if (r < rearRightMotor.GetSelectedSensorVelocity(0) - increment)
  {
    rightOutput = rearRightMotor.GetSelectedSensorVelocity(0) - increment;
  }
  else
  {
    rightOutput = r;
  }

  frontLeftMotor.Set(ControlMode::Follower, 5);
  rearLeftMotor.Set(ControlMode::Velocity, leftOutput);
  frontRightMotor.Set(ControlMode::Follower, 1);
  rearRightMotor.Set(ControlMode::Velocity, rightOutput);
}

void DriveSystem::JoystickPercentDrive(double x, double y)
{
  double l;
  double r;
  if (y > 0.2)
  {
    y = (y - 0.2) * 1 / .8;
  }
  else if (y < -0.2)
  {
    y = (y + 0.2) * 1 / .8;
  }
  else
  {
    y = 0;
  }

  if (x > 0.2)
  {
    x = (x - 0.2) * 1 / .8;
  }
  else if (x < -0.2)
  {
    x = (x + 0.2) * 1 / .8;
  }
  else
  {
    x = 0;
  }

  l = -y + x;
  r = -y - x;

  frontLeftMotor.Set(ControlMode::PercentOutput, l);
  rearLeftMotor.Set(ControlMode::PercentOutput, l);
  frontRightMotor.Set(ControlMode::PercentOutput, r);
  rearRightMotor.Set(ControlMode::PercentOutput, r);
}