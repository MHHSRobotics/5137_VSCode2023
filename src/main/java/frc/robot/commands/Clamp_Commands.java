// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Clamp_Subsystem;
 
public class Clamp_Commands {
  private final Clamp_Subsystem clamp;

  public Clamp_Commands(Clamp_Subsystem clamp) {
    this.clamp = clamp;
  }

  public Command clamp(){
    return new InstantCommand(() -> clamp.clamp());
  }

  /*public Command clampCube(){
    return new InstantCommand(() -> clamp.clamp());
  }*/

  public Command clampRelease(){
    return new InstantCommand(() -> clamp.release());
  }
}
