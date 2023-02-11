package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SmartDashboard_Subsystem extends SubsystemBase {
  public static Double x[] = {0.0, 3.5};
  
  public SmartDashboard_Subsystem() {
    SmartDashboard.putNumberArray("DriveBase", x);
  }

  @Override
  public void periodic() {

  }
}
