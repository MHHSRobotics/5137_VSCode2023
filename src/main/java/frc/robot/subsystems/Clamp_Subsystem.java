// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamp_Subsystem extends SubsystemBase {
  /** Creates a new Clamp_Subsystem. */
  public Clamp_Subsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void clamp() {
    Pneumatics_Subsystem.clampSolenoid.set(true);   
  }

  public void release(){
    Pneumatics_Subsystem.clampSolenoid.set(false);
  }
}
