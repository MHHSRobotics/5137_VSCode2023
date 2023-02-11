package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SmartDashboard_Subsystem extends SubsystemBase {

  public SmartDashboard_Subsystem() {
    SmartDashboard.putData("jMoney's DriveBase", DriveBase_Subsystem.jMoney_Drive);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.updateValues();
  }
}
