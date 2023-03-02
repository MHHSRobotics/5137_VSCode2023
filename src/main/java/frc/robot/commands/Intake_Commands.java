package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Intake_Subystem;

public class Intake_Commands {
    private final Intake_Subystem intake;

    public Intake_Commands (Intake_Subystem intake) {
        this.intake = intake;
    }

    public Command runIntakeForward() {
        return new InstantCommand(() -> {intake.runForward(); intake.extendIntake();});
    }

    public Command runIntakeReverse() {
        return new InstantCommand(() -> {intake.runReverse(); intake.extendIntake();});
    }

    public Command stopIntake() {
        return new InstantCommand(() -> {intake.stop(); intake.retractIntake();});
    }
}