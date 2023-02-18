// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.geometry.Pose2d;
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
public class AutoDrive extends CommandBase {

    Pose2d targetPose;
    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;

    public AutoDrive(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)
    {
      addRequirements(driveBase_Subsystem);

      this.targetPose = targetPose;
      this.driveBase_Subsystem = driveBase_Subsystem;
    }


  @Override 
   public void initialize()
   {}

  @Override
  public void execute()
  {
    motorSpeed = driveBase_Subsystem.autoDrive(targetPose);
    
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(motorSpeed < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}