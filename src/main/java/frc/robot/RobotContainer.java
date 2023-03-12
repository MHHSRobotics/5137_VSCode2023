// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.systems.*;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  public static LED_Subsystem led_Subsystem;
  public static Pneumatics_Subsystem pneumatics_Subsystem;
  public static Drive_Subsystem drive_Subsystem;
  public static Intake_Subystem intake_Subsystem;
  public static Clamp_Subsystem clamp_Subsystem;
  public static Arm_Subsystem arm_Subsystem;
  public static Vision_Subsystem vision_Subsystem;

  //Other Systems
  public static Shuffleboard shuffleboard;
  public static AutoManager autoManager;

  //Commands
  public static Intake_Commands intake_Commands;
  public static Arm_Commands arm_Commands;
  public static Clamp_Commands clamp_Commands;
  public static Pneumatics_Commands pneumatics_Commands;
  public static Drive_Commands drive_Commands;
  public static LED_Commands led_Commands;
  
  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

  public static int LEDpulse;

  public RobotContainer() {
    driverController = new Joystick(0);
    assistController = new Joystick(1);
    configureSystems();
    configureBindings();
  }

  //PNEUMATICS FIRST, SHUFFLEBOARD LAST
  private void configureSystems() {
    configureSubsystems();
    configureCommands();
    shuffleboard = new Shuffleboard();
    autoManager = new AutoManager(drive_Subsystem, intake_Commands, arm_Commands, clamp_Commands);
  }

  public void configureSubsystems() {
    led_Subsystem = new LED_Subsystem();
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    drive_Subsystem = new Drive_Subsystem(driverController);
    intake_Subsystem = new Intake_Subystem();
    arm_Subsystem = new Arm_Subsystem(assistController);
    clamp_Subsystem = new Clamp_Subsystem();
    vision_Subsystem = new Vision_Subsystem();
  }

  public void configureCommands() {
    intake_Commands = new Intake_Commands(intake_Subsystem);
    arm_Commands = new Arm_Commands(arm_Subsystem);
    clamp_Commands = new Clamp_Commands(clamp_Subsystem);
    pneumatics_Commands = new Pneumatics_Commands(pneumatics_Subsystem);
    drive_Commands = new Drive_Commands(drive_Subsystem);
    led_Commands = new LED_Commands(led_Subsystem);
  }

  
  
  public void configureBindings() { 
  //Driver Controller 
    //Right trigger intakes / extends -- retracts when released 
    new Trigger(createBooleanSupplier(driverController, XBOX_Constants.LTPort, XBOX_Constants.RTPort))
    .whileTrue(intake_Commands.runIntakeReverse())
    .onFalse(intake_Commands.stopIntake());

    //Left trigger reverse extends / extends -- retracts when released 
    new Trigger(createBooleanSupplier(driverController, XBOX_Constants.RTPort, XBOX_Constants.LTPort))
    .whileTrue(intake_Commands.runIntakeForward())
    .onFalse(intake_Commands.stopIntake());

    //Start button force compresses (won't work if already at 120 PSI)
    new JoystickButton(driverController, XBOX_Constants.Back)
    .onTrue(pneumatics_Commands.enableCompressor());

    //Back button force stops the compresses 
    new JoystickButton(driverController, XBOX_Constants.Start) //usefull if working on some other component and don't want it running  
    .onTrue(pneumatics_Commands.disableCompressor()); 

    //Middle down dpad aligns to april tag
    new POVButton(driverController, XBOX_Constants.DownDPad)
    .onTrue(drive_Commands.tagDrive(vision_Subsystem.getNearestAlign("middle", drive_Subsystem.poseEstimator.getEstimatedPosition())));

    //Left dpad aligns to cone left of april tag (from drivers pov not cameras)
    new POVButton(driverController, XBOX_Constants.LeftDPad)
    .onTrue(drive_Commands.tagDrive(vision_Subsystem.getNearestAlign("left", drive_Subsystem.poseEstimator.getEstimatedPosition())));

    //Right dpad aligns to cone right of april tag (from drivers pov not camera)
    new POVButton(driverController, XBOX_Constants.RightDPad)
    .onTrue(drive_Commands.tagDrive(vision_Subsystem.getNearestAlign("right", drive_Subsystem.poseEstimator.getEstimatedPosition())));

    
    new JoystickButton(driverController, XBOX_Constants.AButton)
    .onTrue(new InstantCommand(() -> drive_Subsystem.driveBrake()))
    .onFalse(new InstantCommand(() -> drive_Subsystem.driveCoast()));

    //Cube leds for signaling
    new JoystickButton(driverController, XBOX_Constants.XButton)
    .onTrue(led_Commands.cubeLEDS());
    
    //Cone leds for signaling
    new JoystickButton(driverController, XBOX_Constants.YButton)
    .onTrue(led_Commands.coneLEDS());

    /*new JoystickButton(driverController, XBOX_Constants.BButton)
    .onTrue(intake_Commands.justRetract());*/


  //Assistant Controller
    //right trigger clamps
    new Trigger(createBooleanSupplier(assistController, XBOX_Constants.RTPort, XBOX_Constants.LTPort))  //right trigger 
    .whileTrue(clamp_Commands.clamp());

    //left trigger releases clamp
    new Trigger(createBooleanSupplier(assistController, XBOX_Constants.LTPort, XBOX_Constants.RTPort))  //left trigger 
    .whileTrue(clamp_Commands.clampRelease());

    //Up on Dpad moves to intake 
    new POVButton(assistController, XBOX_Constants.UpDPad)
    .onTrue(arm_Commands.moveToIntake());

    //Down on Dpad moves to hypbrid location / the floor 
    new POVButton(assistController, XBOX_Constants.DownDPad)
    .onTrue(arm_Commands.moveToHybrid());

    //B -- midle cube 
    new JoystickButton(assistController, XBOX_Constants.BButton)
    .onTrue(arm_Commands.moveToMiddleCube());

    //A -- middle cone 
    new JoystickButton(assistController, XBOX_Constants.AButton)
    .onTrue(arm_Commands.moveToMiddleCone());

    //Y -- top cube
    new JoystickButton(assistController, XBOX_Constants.YButton)
    .onTrue(arm_Commands.moveToTopCube());

    //X -- top cone 
    new JoystickButton(assistController, XBOX_Constants.XButton)
    .onTrue(arm_Commands.moveToTopCone());
  }

  public void runAuto() {
    autoManager.runAuto(shuffleboard.getAuto());
  }

  public void runLEDS() {
    led_Subsystem.pulsingCG(25, 10);
  }

  public void runLEDSAuto() {
    led_Subsystem.solidCG(1);
  }

  public void runLEDSTeleOp() {
    led_Subsystem.runLEDS();
  }

  //required port is the joystick you are currecntly attempting to use 
  //dependent port is the joytick we're checking against, to make sure you're not breaking the robot 
  private BooleanSupplier createBooleanSupplier(Joystick controller, int requiredPort, int dependentPort) {
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
