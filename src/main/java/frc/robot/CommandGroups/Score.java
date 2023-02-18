// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;
//import frc.robot.commands.Arm_Commands.TopConePreset;
import frc.robot.commands.Clamp_Commands.ClampCone;
import frc.robot.commands.Clamp_Commands.ClampOpen;
import frc.robot.commands.Intake_Commands.IntakeExtend;
import frc.robot.subsystems.DriveBase_Subsystem;


public class Score extends SequentialCommandGroup {
  public Score(Command placePreset) {
    addRequirements(RobotContainer.driveBase_Subsystem);
    addRequirements(RobotContainer.arm_Subsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new IntakeExtend(),new ClampCone(), placePreset,  new ClampOpen());
  }
}
