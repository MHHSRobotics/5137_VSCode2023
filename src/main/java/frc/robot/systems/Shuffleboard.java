package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive_Subsystem;

public class Shuffleboard extends SubsystemBase {
    public static SendableChooser<String> autoChoice = new SendableChooser<>();
    public static SendableChooser<Boolean> autoScoring = new SendableChooser<>();
    public static SendableChooser<Boolean> ledsEnabled = new SendableChooser<>();

    public Shuffleboard() {
        configureSendableString(autoChoice, "None", "leftMobility", "middleEngage", "rightMobility", "middleCombo" , "TimedMobility", "TimedEngage", "TimedMobilityEngage");
        configureSendableBoolean(autoScoring, false);
        configureSendableBoolean(ledsEnabled, true);

        SmartDashboard.putData("Auto Choice", autoChoice);
        SmartDashboard.putData("Auto Scoring", autoScoring);
        SmartDashboard.putData("LEDs Enabled", ledsEnabled);
        SmartDashboard.putData("jMoneyDrive", Drive_Subsystem.jMoneyDrive);

        update();
    }

    @Override
    public void periodic() {
        update();
    }

    public String getAuto() {
        if (autoScoring.getSelected()) {
            return autoChoice.getSelected()+"@Score";
        } else {
            return autoChoice.getSelected()+"@None";
        }
    }

    public Boolean getLEDsEnabled() {
        return ledsEnabled.getSelected();
    }

    private void update() {
        SmartDashboard.putBoolean("Punch Solenoids", RobotContainer.punch_Subsystem.getSolenoidsActive());
        SmartDashboard.putBoolean("Compressor", RobotContainer.punch_Subsystem.getCompActive());
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
