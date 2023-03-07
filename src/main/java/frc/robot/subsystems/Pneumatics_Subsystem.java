// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  public static Compressor comp;
  public static Solenoid intakeSolenoid;
  public static Solenoid clampSolenoid;
  public static Solenoid feetSolenoid;
  private boolean compressed;
 
  public Pneumatics_Subsystem() {
    comp = new Compressor(PneumaticsModuleType.REVPH);
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.intakeSolChannel);
    clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.clampSolChannel);
    feetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.feetSolChannel);
 comp.enableAnalog(110, 120);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void compress(boolean active) {
    if (active) {
      comp.enableAnalog(110, 120);
      compressed = true;
    } else {
      comp.enableAnalog(0, 120);
      compressed = false;
    } 
  } 
  
  public boolean getCompressed()
  {
    return compressed;
  }


}
