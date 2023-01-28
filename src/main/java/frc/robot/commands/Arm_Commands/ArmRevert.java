// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class ArmRevert extends CommandBase {
  /** Creates a new ArmRevert. */
  public ArmRevert() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.arm_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.arm_Subsystem.armExtend(-Arm_Subsystem.currentExtension);
    RobotContainer.arm_Subsystem.armRotate(-Arm_Subsystem.currentRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm_Subsystem.currentExtension = 0;
    Arm_Subsystem.currentRotation = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
