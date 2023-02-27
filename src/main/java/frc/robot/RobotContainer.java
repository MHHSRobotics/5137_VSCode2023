// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.constants.Controller_Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  public static Pneumatics_Subsystem pneumatics_Subsystem;
  public static Drive_Subsystem drive_Subsystem;
  public static Intake_Subystem intake_Subsystem;
  public static Clamp_Subsystem clamp_Subsystem;
  public static Shuffleboard_Subsystem shuffleboard_Subsystem;

  //Commands
  public static Intake_Commands intake_Commands;
  public static Clamp_Commands clamp_Commands;
  public static Pneumatics_Commands pneumatics_Commands;
  
  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  public RobotContainer() {
    driverController = new Joystick(0);
    assistController = new Joystick(1);
    configureSubsystems();
    configureCommands();
    configureBindings();
  }

  //PNEUMATICS FIRST, SHUFFLEBOARD LAST
  public void configureSubsystems() {
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    drive_Subsystem = new Drive_Subsystem(driverController);
    intake_Subsystem = new Intake_Subystem();
    clamp_Subsystem = new Clamp_Subsystem();
    shuffleboard_Subsystem = new Shuffleboard_Subsystem();
  }

  public void configureCommands() {
    intake_Commands = new Intake_Commands(intake_Subsystem);
    clamp_Commands = new Clamp_Commands(clamp_Subsystem);
    pneumatics_Commands = new Pneumatics_Commands(pneumatics_Subsystem);
  }
  
  public void configureBindings() {
    new Trigger(createBooleanSupplier(driverController, PS4_Constants.LTPort, PS4_Constants.RTPort))
    .whileTrue(intake_Commands.runIntakeReverse())
    .onFalse(intake_Commands.stopIntake());

    new Trigger(createBooleanSupplier(driverController, PS4_Constants.RTPort, PS4_Constants.LTPort))
    .whileTrue(intake_Commands.runIntakeForward())
    .onFalse(intake_Commands.stopIntake());

    new JoystickButton(driverController, PS4_Constants.SharePort)
    .onTrue(pneumatics_Commands.enableCompressor());

    new JoystickButton(driverController, PS4_Constants.OptionsPort)
    .onTrue(pneumatics_Commands.disableCompressor());

    new Trigger(createBooleanSupplier(assistController, PS4_Constants.RTPort, PS4_Constants.LTPort))
    .whileTrue(clamp_Commands.clampCone())
    .onFalse(clamp_Commands.clampRelease());

    new Trigger(createBooleanSupplier(assistController, PS4_Constants.LTPort, PS4_Constants.RTPort))
    .whileTrue(clamp_Commands.clampCube())
    .onFalse(clamp_Commands.clampRelease());
  }

  //required port is the joystick you are currecntly attempting to use 
  //dependent port is the joytick we're checking against, to make sure you're not breaking the robot 
  public BooleanSupplier createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort) {
    BooleanSupplier supply;
    supply = () -> {
      if (controller != null) {
        if (controller.getRawAxis(requiredPort) > 0.1 && controller.getRawAxis(dependentPort) < 0.1) {
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    };
    return supply;
  }
}
