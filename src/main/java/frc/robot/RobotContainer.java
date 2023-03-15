// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.systems.*;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
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
  public static Drive_Subsystem drive_Subsystem;
  public static Arm_Subsystem arm_Subsystem;
  public static Vision_Subsystem vision_Subsystem;

  //Other Systems
  public static Shuffleboard shuffleboard;
  public static AutoManager autoManager;

  //Commands
  public static Arm_Commands arm_Commands;
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
    autoManager = new AutoManager(drive_Subsystem, arm_Commands);
  }

  public void configureSubsystems() {
    led_Subsystem = new LED_Subsystem();
    drive_Subsystem = new Drive_Subsystem(driverController);
    arm_Subsystem = new Arm_Subsystem(assistController);
    vision_Subsystem = new Vision_Subsystem();
  }

  public void configureCommands() {
    arm_Commands = new Arm_Commands(arm_Subsystem);
    drive_Commands = new Drive_Commands(drive_Subsystem);
    led_Commands = new LED_Commands(led_Subsystem);
  }

  
  
  public void configureBindings() { 
  //Driver Controller 
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

  //Assistant Controller
    //moves to start/intake position
    new JoystickButton(assistController, XBOX_Constants.YButton)
    .onTrue(arm_Commands.moveToStart());

    //flings it 
    new JoystickButton(assistController, XBOX_Constants.AButton)
    .onTrue(arm_Commands.fling());

     //Cube leds for signaling
     new JoystickButton(assistController, XBOX_Constants.XButton)
     .onTrue(led_Commands.cubeLEDS());
     
     //Cone leds for signaling
     new JoystickButton(assistController, XBOX_Constants.YButton)
     .onTrue(led_Commands.coneLEDS());
  }  

  public void runAuto() {
    autoManager.runAuto(shuffleboard.getAuto());
  }

  public void startTimers() {
    led_Subsystem.startTimer();
  }


  /* 
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
  */
}
