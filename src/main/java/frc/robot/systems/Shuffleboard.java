package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.subsystems.Drive_Subsystem;
import frc.robot.objects.AutoData;

public class Shuffleboard extends SubsystemBase {
    SendableChooser<String> autoColumn = new SendableChooser<>();
    SendableChooser<String> autoChoice = new SendableChooser<>();
    SendableChooser<Boolean> autoMobility = new SendableChooser<>();
    SendableChooser<Boolean> autoEngage = new SendableChooser<>();

    public Shuffleboard() {
        configureSendableString(autoColumn, "Middle", "Left", "Middle", "Right");
        configureSendableString(autoChoice, "None", "None", "SingleScore");
        configureSendableBoolean(autoMobility, false);
        configureSendableBoolean(autoEngage, false);

        SmartDashboard.putData("Auto Position", autoColumn);
        SmartDashboard.putData("Auto Choice", autoChoice);
        SmartDashboard.putData("Mobility", autoMobility);
        SmartDashboard.putData("Engage", autoEngage);
        SmartDashboard.putData("jMoneyDrive", Drive_Subsystem.jMoneyDrive);

        update();
    }

    @Override
    public void periodic() {
        update();
    }

    public AutoData getAuto() {
       return new AutoData(autoColumn.getSelected(), autoChoice.getSelected(), autoMobility.getSelected(), autoEngage.getSelected());
    }

    private void update() {
        //SmartDashboard.putNumber("Arm Rotate Motor", RobotContainer.arm_Subsystem.getRotationSpeed());
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
