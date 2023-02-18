package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SmartDashboard_Subsystem extends SubsystemBase {
  public Boolean manualArmEnabled = false;

  SendableChooser<String> driverControlChooser = new SendableChooser<>();
  SendableChooser<String> assistControlChooser = new SendableChooser<>();
  SendableChooser<String> armPreset = new SendableChooser<>();
  SendableChooser<Boolean> manualArm = new SendableChooser<>();

  Mechanism2d armSim = new Mechanism2d(4, 4, new Color8Bit(Color.kBlack));
  MechanismRoot2d armSimRoot = armSim.getRoot("ArmBase", 2, 0);
  MechanismLigament2d armSimPart = armSimRoot.append(new MechanismLigament2d("Arm", 2, 90));

  public SmartDashboard_Subsystem() {
    driverControlChooser.setDefaultOption("XBOX", "xbox");
    driverControlChooser.addOption("XBOX", "xbox");
    driverControlChooser.addOption("PLAY_STATION", "ps4");

    assistControlChooser.setDefaultOption("XBOX", "xbox");
    assistControlChooser.addOption("XBOX", "xbox");
    assistControlChooser.addOption("PLAY_STATION", "ps4");

    armPreset.setDefaultOption("None", "None");
    armPreset.addOption("None", "None");
    armPreset.addOption("Hybrid", "Hybrid");
    armPreset.addOption("Middle Cube", "Middle Cube");
    armPreset.addOption("Middle Cone", "Middle Cone");
    armPreset.addOption("Top Cube", "Top Cube");
    armPreset.addOption("Top Cone", "Top Cone");

    manualArm.setDefaultOption("Disabled", false);
    manualArm.addOption("Disabled", false);
    manualArm.addOption("Enabled", true);
    
    SmartDashboard.putData("Driver Controller Type", driverControlChooser);
    SmartDashboard.putData("Assist Controller Type", assistControlChooser);
    SmartDashboard.putData("Manual Arm Preset", armPreset);
    SmartDashboard.putData("Manual Arm", manualArm);
    SmartDashboard.putData("jMoney's DriveBase", DriveBase_Subsystem.jMoney_Drive);
    SmartDashboard.putData("ArmSim", armSim);

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

  public Double getArmRotate() {
    switch (armPreset.getSelected()) {
      case ("None"): {return 0.0;}
      case ("Hybrid"): {return Constants.hybridRotation;}
      case ("Middle Cube"): {return Constants.middleCubeRotation;}
      case ("Middle Cone"): {return Constants.middleConeRotation;}
      case ("Top Cube"): {return Constants.topCubeRotation;}
      case ("Top Cone"): {return Constants.topConeRotation;}
      default: {return 0.0;}
    }
  }

  public Double getArmExtension() {
    switch (armPreset.getSelected()) {
      case ("None"): {return 0.0;}
      case ("Hybrid"): {return Constants.hybridExtension;}
      case ("Middle Cube"): {return Constants.middleCubeExtension;}
      case ("Middle Cone"): {return Constants.middleConeExtension;}
      case ("Top Cube"): {return Constants.topCubeExtension;}
      case ("Top Cone"): {return Constants.topConeExtension;}
      default: {return 0.0;}
    }
  }

  private void updateArmSim() {
    armSimPart.setLength(2 + (Arm_Subsystem.extendEncoder.getDegrees()/50));
    armSimPart.setAngle(90 + Arm_Subsystem.rotateEncoder.getDegrees());
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
    updateArmSim();
    manualArmEnabled = manualArm.getSelected();
  }
}
