package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Clamp_Subsystem extends SubsystemBase {

  public Clamp_Subsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void clamp() {
    RobotContainer.pneumatics_Subsystem.enableClamp();
  }

  public void release(){
    RobotContainer.pneumatics_Subsystem.disableClamp();
  }
}
