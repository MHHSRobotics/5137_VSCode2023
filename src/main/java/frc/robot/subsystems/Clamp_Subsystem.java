package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;

public class Clamp_Subsystem extends SubsystemBase {
  boolean clampStatus;

  public Clamp_Subsystem() {
    clampStatus = false; //true = open, false = closed
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void clamp() {
    RobotContainer.pneumatics_Subsystem.enableClamp();
    clampStatus = true;
  }

  public void release(){
    RobotContainer.pneumatics_Subsystem.disableClamp();
    clampStatus = false;
  }

  public void toggle(){
    RobotContainer.pneumatics_Subsystem.toggleClamp();
  }

  /*public boolean clampStatus(){
    return clampStatus;
  }*/
}
