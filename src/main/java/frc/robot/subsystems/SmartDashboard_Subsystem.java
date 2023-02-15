package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class SmartDashboard_Subsystem extends SubsystemBase {
  SendableChooser<String> driverControlChooser = new SendableChooser<>();
  SendableChooser<String> assistControlChooser = new SendableChooser<>();

  public SmartDashboard_Subsystem() {
    driverControlChooser.setDefaultOption("XBOX", "xbox");
    driverControlChooser.addOption("XBOX", "xbox");
    driverControlChooser.addOption("PLAY_STATION", "ps4");

    assistControlChooser.setDefaultOption("XBOX", "xbox");
    assistControlChooser.addOption("XBOX", "xbox");
    assistControlChooser.addOption("PLAY_STATION", "ps4");
    
    SmartDashboard.putData("Driver Controller Type", driverControlChooser);
    SmartDashboard.putData("Assist Controller Type", assistControlChooser);

    SmartDashboard.putData("jMoney's DriveBase", DriveBase_Subsystem.jMoney_Drive);
    update();
  }

  @Override
  public void periodic() {
    update();
  }

  public String selectDriverController() {
    return driverControlChooser.getSelected();
  }

  public String selectAssistController() {
    return assistControlChooser.getSelected();
  }

  private void update() {
    SmartDashboard.putNumber("IntakeMotor", Intake_Subsystem.intakeMotor.get());
    SmartDashboard.putNumber("RotateMotor", Arm_Subsystem.armRotateMotor.get());
    SmartDashboard.putNumber("ExtendMotor", Arm_Subsystem.armExtendMotor.get());
    SmartDashboard.putNumber("RotateEncoder", Arm_Subsystem.rotateEncoder.getDegrees()); //Set these both to getPosition
    SmartDashboard.putNumber("ExtendEncoder", Arm_Subsystem.extendEncoder.getDegrees()); //once the bot actually works
    SmartDashboard.putString("Arm Preset", Arm_Subsystem.activePreset);
    SmartDashboard.putBoolean("IntakeSolenoid", Pneumatics_Subsystem.intakeSolenoid.get());
    SmartDashboard.putBoolean("ClampSolenoid", Pneumatics_Subsystem.clampSolenoid.get());
    SmartDashboard.putBoolean("Compressor", Pneumatics_Subsystem.comp.isEnabled()); 
  }
}
