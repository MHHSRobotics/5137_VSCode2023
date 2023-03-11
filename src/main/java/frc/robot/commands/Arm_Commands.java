package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.Arm_Subsystem;
import frc.robot.constants.Arm_Constants;

public class Arm_Commands {
    private final Arm_Subsystem arm;
    
    public Arm_Commands(Arm_Subsystem arm) {
        this.arm = arm;
    }

    public Command moveToIntake() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.armIntakeRotation, Arm_Constants.armIntakeExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command moveToHybrid() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.hybridRotation, Arm_Constants.hybridExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command moveToMiddleCube() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.middleCubeRotation, Arm_Constants.middleCubeExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command moveToMiddleCone() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.middleConeRotation, Arm_Constants.middleConeExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command moveToTopCube() {
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.topCubeRotation, Arm_Constants.topCubeExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command moveToTopCone() {
        System.out.println("move arm -- command");
        return new FunctionalCommand(
            () -> {},
            () -> {arm.moveArm(Arm_Constants.topConeRotation, Arm_Constants.topConeExtension);},
            arm.endCommand,
            arm.isFinished,
            arm);
    }

    public Command stopArm() {
        return new InstantCommand(() -> {arm.stopArm();});
    }
}
