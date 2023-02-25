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
  public static Drive drive_Subsystem;
  public static Intake_Subystem intake_Subsystem;
  public static Pneumatics_Subsystem pneumatics_Subsystem;
  public static Clamp_Subsystem clamp_Subsystem;

  //Commands
  public static Intake_Commands intake_Commands;
  public static Clamp_Commands clamp_Commands;


  //Controller Triggers
  public static Trigger driver_LT;
  public static Trigger driver_RT;
  public static Trigger assist_LT;
  public static Trigger assist_RT;

  //Controller Buttons
  public static JoystickButton driver_AButton;
  
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

  public void configureSubsystems() {
    drive_Subsystem = new Drive(driverController);
    intake_Subsystem = new Intake_Subystem();
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    clamp_Subsystem = new Clamp_Subsystem();
  }

  public void configureCommands() {
    intake_Commands = new Intake_Commands(intake_Subsystem);
    clamp_Commands = new Clamp_Commands(clamp_Subsystem);
  }
  
  public void configureBindings() {
    driver_LT = new Trigger(createBooleanSupplier(driverController, XBOX_Constants.LTPort, XBOX_Constants.RTPort));
    driver_RT = new Trigger(createBooleanSupplier(driverController, XBOX_Constants.RTPort, XBOX_Constants.LTPort));

    driver_LT.whileTrue(intake_Commands.runIntakeReverse());
    driver_LT.onFalse(intake_Commands.stopIntake());

    driver_RT.whileTrue(intake_Commands.runIntakeForward());
    driver_RT.onFalse(intake_Commands.stopIntake());

    driver_AButton = new JoystickButton(driverController, XBOX_Constants.APort);

    assist_RT = new Trigger(createBooleanSupplier(assistController, XBOX_Constants.RTPort, XBOX_Constants.LTPort));
    assist_RT.whileTrue(clamp_Commands.clampCone());
    assist_RT.onFalse(clamp_Commands.clampRelease());

    assist_LT = new Trigger(createBooleanSupplier(assistController, XBOX_Constants.LTPort, XBOX_Constants.RTPort));
    assist_LT.whileTrue(clamp_Commands.clampCube());
    assist_LT.onFalse(clamp_Commands.clampRelease());
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
