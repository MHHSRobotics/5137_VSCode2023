package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shuffleboard_Subsystem extends SubsystemBase {
    SendableChooser<String> autoChooser = new SendableChooser<>();
    SendableChooser<String> armPreset = new SendableChooser<>();
    SendableChooser<Boolean> manualArm = new SendableChooser<>();

    public Shuffleboard_Subsystem() {
        configureSendableString(autoChooser, "AutoOne", "AutoOne", "AutoTwo", "AutoThree");
        configureSendableString(armPreset, "None", "None", "Hybrid", "MidCube", "MidCone", "TopCube", "TopCone");
        configureSendableBoolean(manualArm, false);

        SmartDashboard.putData("Auto Choice", autoChooser);
        SmartDashboard.putData("Arm Preset", armPreset);
        SmartDashboard.putData("Manual Arm", manualArm);
        SmartDashboard.putData("jMoneyDrive", Drive_Subsystem.jMoneyDrive);

        update();
    }

    

    @Override
    public void periodic() {
        update();
    }

    private void update() {
        SmartDashboard.putNumber("Intake Motor", RobotContainer.intake_Subsystem.getIntakeSpeed());
        SmartDashboard.putNumber("Arm Rotate Motor", RobotContainer.arm_Subsystem.getRotationSpeed());
        SmartDashboard.putNumber("Arm Extend Motor", RobotContainer.arm_Subsystem.getExtensionSpeed());
        SmartDashboard.putNumber("Arm Rotate Position", RobotContainer.arm_Subsystem.getRotationPosition());
        SmartDashboard.putNumber("Arm Extend Position", RobotContainer.arm_Subsystem.getExtensionPosition());
        SmartDashboard.putBoolean("Intake Solenoid", RobotContainer.pneumatics_Subsystem.getIntakeEnabled());
        SmartDashboard.putBoolean("Clamp Solenoid", RobotContainer.pneumatics_Subsystem.getClampEnabled());
    }

    private void configureSendableString(SendableChooser<String> chooser, String kDefault, String... kOptions) {
        chooser.setDefaultOption(kDefault, kDefault);
        for (String Option:kOptions) {
            chooser.addOption(Option, Option);
        }
    }

    private void configureSendableBoolean(SendableChooser<Boolean> chooser, Boolean kDefault) {
        if (kDefault) {
            chooser.setDefaultOption("Enabled", true);
        } else {
            chooser.setDefaultOption("Disabled", false);
        }
        chooser.addOption("Enabled", true);
        chooser.addOption("Disabled", false);
    }
}
