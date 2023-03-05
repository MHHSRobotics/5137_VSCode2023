// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Pneumatics_Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  private static Compressor comp;
  private static Solenoid intakeSolenoid;
  private static Solenoid clampSolenoid;
  private static Solenoid feetSolenoid;
  private static ModuleType compModule;

  public Pneumatics_Subsystem() {
    comp = new Compressor(6, PneumaticsModuleType.REVPH); //Change to the other type later
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.intakeSolChannel);
    clampSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.clampSolChannel);
    feetSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Pneumatics_Constants.feetSolChannel);
    
    
    
    //comp.enableAnalog(110, 120);
    comp.enableDigital();
  }

  @Override
  public void periodic() {

    //System.out.println(comp.getAnalogVoltage()); 
    System.out.println(comp.getPressure());
    


  }

  public void enableCompressor() {
    //comp.enableAnalog(110, 120);
    comp.enableDigital();
  }

  /*public void disableCompressor() {
    comp.disable();
  }*/

  public void enableIntake() {
    intakeSolenoid.set(true);
  }

  public void disableIntake() {
    intakeSolenoid.set(false);
  }

  public void enableClamp() {
    clampSolenoid.set(true);
  }

  public void disableClamp() {
    clampSolenoid.set(false);
  }

  public void enableFeet() {
    feetSolenoid.set(true);
  }

  public void disableFeet() {
    feetSolenoid.set(false);
  }

  public boolean getIntakeEnabled() {
    return intakeSolenoid.get();
  }

  public boolean getClampEnabled() {
    return clampSolenoid.get();
  }

  public boolean getFeetEnabled() {
    return feetSolenoid.get();
  }

  public boolean getCompressorEnabled() {
    return comp.isEnabled();
  }
}
 