// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
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
  Intake intake_Subsystem;

  //Commands
  Intake_Commands intake_Commands;

  //Controller
  public static Trigger driver_LT;
  public static Trigger driver_RT;
  
  //Controllers
  Joystick driverController;
  Joystick assistController;

  public RobotContainer() {
    driverController = new Joystick(0);
    assistController = new Joystick(1);
    configureSubsystems();
    configureCommands();
    configureBindings();
  }

  public void configureSubsystems() {
    intake_Subsystem = new Intake();
  }

  public void configureCommands() {
    intake_Commands = new Intake_Commands(intake_Subsystem);
  }
  
  public void configureBindings() {
    driver_LT = new Trigger(createBooleanSupplier(driverController, PS4_Constants.LTPort, PS4_Constants.RTPort));
    driver_RT = new Trigger(createBooleanSupplier(driverController, PS4_Constants.RTPort, PS4_Constants.LTPort));

    driver_LT.whileTrue(intake_Commands.runIntakeReverse());
    driver_LT.onFalse(intake_Commands.stopIntake());

    driver_RT.whileTrue(intake_Commands.runIntakeForward());
    driver_RT.onFalse(intake_Commands.stopIntake());
  }

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
