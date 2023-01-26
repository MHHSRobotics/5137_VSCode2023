// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics_Subsystem extends SubsystemBase {
  public static Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  public static SolenoidSim intakeSolenoid = new SolenoidSim(PneumaticsModuleType.CTREPCM, Constants.intakeSolChannel);
  public static SolenoidSim clampSolenoid = new SolenoidSim(PneumaticsModuleType.CTREPCM, Constants.clampSolChannel);
  public static SolenoidSim feetSolenoid = new SolenoidSim(PneumaticsModuleType.CTREPCM, Constants.feetSolChannel);

 
  public Pneumatics_Subsystem() {
    
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
      comp.enableDigital();
    } else {
      comp.disable();
    } 
  } 

}