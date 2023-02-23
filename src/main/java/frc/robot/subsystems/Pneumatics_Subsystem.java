// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Pneumatics_Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  public static Solenoid intakeSolenoid;
  public static Solenoid clampSolenoid;
  public static Solenoid feetSolenoid;




  /** Creates a new Pneumatics. */
  public Pneumatics_Subsystem() {
    final Compressor comp = new Compressor(PneumaticsModuleType.REVPH);

    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.intakeSolChannel);
    clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.clampSolChannel);
    feetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.feetSolChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run 

  }
}
