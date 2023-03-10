// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Compressor_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics_Subsystem;

public class CompressorOn extends CommandBase {
  /** Creates a new CompressorOn. */
  Pneumatics_Subsystem pneumatics_Subsystem;

  public CompressorOn(Pneumatics_Subsystem pneumatics_Subsystem) {
    this.pneumatics_Subsystem = pneumatics_Subsystem;

    addRequirements(pneumatics_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pneumatics_Subsystem.compress(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pneumatics_Subsystem.getCompressed())
    {
      return true;
    }
    return false;
  }
}
