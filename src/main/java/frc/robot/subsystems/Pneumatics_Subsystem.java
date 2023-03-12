// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Pneumatics_Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  private static Compressor comp;
  private static PneumaticHub hub;  
  private static Solenoid intakeSolenoid;
  private static Solenoid clampSolenoid;
  private static Solenoid feetSolenoid;

  public Pneumatics_Subsystem() {
    
    hub = new PneumaticHub(6);
    
    comp = new Compressor(PneumaticsModuleType.REVPH); //Change to the other type later
    intakeSolenoid = hub.makeSolenoid(Pneumatics_Constants.intakeSolChannel);
    clampSolenoid = hub.makeSolenoid(Pneumatics_Constants.clampSolChannel);
    
    comp.enableDigital();
  }

  @Override
  public void periodic() {

  }

  //These don't work atm
  public void enableCompressor() { //get rid of this if it refuses to compress 
    comp.enableDigital();
  }

  public void disableCompressor() {
    comp.disable();
  }

  public void enableIntake() {
    intakeSolenoid.set(true);
  }

  public void disableIntake() {
    intakeSolenoid.set(false);
  }

  public void enableClamp() {
    clampSolenoid.set(false); //worked before cause inverted triggers, this is proper fix
  }

  public void disableClamp() {
    clampSolenoid.set(true);
  }

  public void toggleClamp(){
    clampSolenoid.toggle();
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
