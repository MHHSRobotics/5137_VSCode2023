package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Arm_Subsystem;
import frc.robot.constants.Arm_Constants;

public class Arm_Commands {
    private final Arm_Subsystem arm;
    
    public Arm_Commands(Arm_Subsystem arm) {
        this.arm = arm;
    }

    public Command moveToStart() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.startPosition);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command fling() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.flingEndPosition);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

}
