package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Pneumatics_Subsystem;

public class Pneumatics_Commands {
  private final Pneumatics_Subsystem pneumatics;

  public Pneumatics_Commands(Pneumatics_Subsystem pneumatics) {
    this.pneumatics = pneumatics;
  }

  public Command enableCompressor() {
    return new InstantCommand(() -> pneumatics.enableCompressor());
  }

  public Command disableCompressor() {
    return new InstantCommand(() -> pneumatics.disableCompressor());
  }
}
