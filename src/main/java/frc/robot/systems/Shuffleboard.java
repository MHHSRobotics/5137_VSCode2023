package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive_Subsystem;

public class Shuffleboard extends SubsystemBase {
    SendableChooser<String> autoPosition = new SendableChooser<>();
    SendableChooser<Boolean> autoScore = new SendableChooser<>();
    SendableChooser<Boolean> autoMobility = new SendableChooser<>();
    SendableChooser<Boolean> autoEngage = new SendableChooser<>();
    SendableChooser<Boolean> autoIntake = new SendableChooser<>();


    public Shuffleboard() {
        configureSendableString(autoPosition, "middle", "left", "middle", "right");

        SmartDashboard.putData("Auto Position", autoPosition);
        SmartDashboard.putData("jMoneyDrive", Drive_Subsystem.jMoneyDrive);

        update();
    }

    @Override
    public void periodic() {
        update();
    }

    public String getAutoChoice() {
       return "hi";
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
