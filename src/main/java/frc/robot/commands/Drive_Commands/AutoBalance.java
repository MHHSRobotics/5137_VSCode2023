// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;


/**
 * Drive the robot.
 * 
 * Left stick controls the direction and speed of translation.
 * Right trigger will slow the robot down as an analog input.
 * 
 * Right stick controls the rate of rotation.
 *
 */
public class AutoBalance extends CommandBase {

    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;
    PigeonIMU gyro;
    double gyroAngle;


    public AutoBalance(DriveBase_Subsystem driveBase_Subsystem)
    {
        this.driveBase_Subsystem = driveBase_Subsystem;
        gyro = DriveBase_Subsystem.gyro;
        addRequirements(driveBase_Subsystem);
    }


  @Override 
   public void initialize()
   {}

  @Override
  public void execute()
  {
    motorSpeed = driveBase_Subsystem.autoBalance();
    gyroAngle = gyro.getPitch();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(motorSpeed < 0.05 && Math.abs(gyroAngle) < 2)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}