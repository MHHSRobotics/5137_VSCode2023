// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public PneumaticsSubsystem() {
    final Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);

    final Solenoid intakSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intakeSolChannel);
    final Solenoid clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.clampSolChannel);
    final Solenoid telescopSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.telscopeSolChannel);
    final Solenoid feetSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.feetSolChannel);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
