package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Intake;

public class Intake_Commands {
    private final Intake intake;

    public Intake_Commands (Intake intake) {
        this.intake = intake;
    }

    public Command runIntakeForward() {
        return new InstantCommand(intake::runForward);
    }

    public Command runIntakeReverse() {
        return new InstantCommand(intake::runReverse);
    }

    public Command stopIntake() {
        return new InstantCommand(intake::stop);
    }
}