// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.DriveBase_Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  public static DriveBase_Subsystem driveBase_Subsystem;
  public static AprilTagSubsystem aprilTagSubsystem;

  public RobotContainer() {
    //Subsystems
    driveBase_Subsystem = new DriveBase_Subsystem();
    aprilTagSubsystem = new AprilTagSubsystem();

    // Configure the trigger bindings
    configureBindings();
  }
  private void configureBindings() {
  }
}
