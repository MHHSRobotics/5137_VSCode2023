package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Arm_Subystem;
import frc.robot.constants.Arm_Constants;

public class Arm_Commands {
    private final Arm_Subystem arm;
    
    public Arm_Commands(Arm_Subystem arm) {
        this.arm = arm;
    }

    public Command moveToParallel() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(0);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command fling() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.flingArm();},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

}
