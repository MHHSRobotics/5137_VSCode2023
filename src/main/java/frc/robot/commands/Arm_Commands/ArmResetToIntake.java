// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ArmResetToIntake extends CommandBase {
  /** Creates a new ArmResetToIntake. */
  public ArmResetToIntake() {
    addRequirements(RobotContainer.arm_Subsystem);
    addRequirements(RobotContainer.pneumatics_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (RobotContainer.pneumatics_Subsystem.intakeSolenoid.get()){   //ensures intake is extended before moving arm  
      RobotContainer.arm_Subsystem.moveArm(Constants.armIntakeRotation, Constants.armIntakeExtention);
      RobotContainer.arm_Subsystem.activePreset = "None";
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
