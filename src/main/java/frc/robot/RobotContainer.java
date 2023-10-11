// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.systems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
import frc.robot.constants.Controller_Constants;

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
  public static Punch_Subystem punch_Subsystem;

  //Other Systems
  public static Shuffleboard shuffleboard;
  public static AutoManager autoManager;

  //Commands
  public static Drive_Commands drive_Commands;
  public static LED_Commands led_Commands;
  public static Punch_Commands punch_Commands;
  
  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;

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
    //autoManager = new AutoManager(drive_Subsystem, punch_Commands);
  }

  public void configureSubsystems() {
    led_Subsystem = new LED_Subsystem();
    drive_Subsystem = new Drive_Subsystem(driverController);
    punch_Subsystem = new Punch_Subystem();
  }

  public void configureCommands() {
    drive_Commands = new Drive_Commands(drive_Subsystem);
    led_Commands = new LED_Commands(led_Subsystem);
    punch_Commands = new Punch_Commands(punch_Subsystem);
  }
  
  public void configureBindings() {
    
    
    new JoystickButton(driverController, Controller_Constants.CircleButton)
    .onTrue(drive_Commands.setBrake(true))
    .onFalse(drive_Commands.setBrake(false));

    /*
    new JoystickButton(driverController, Controller_Constants.CircleButton)
    .onTrue(drive_Commands.balance());*/

     //Cube leds for signaling
     new JoystickButton(driverController, Controller_Constants.SquareButton)
     .onTrue(led_Commands.cubeLEDS());
     
     //Cone leds for signaling
     new JoystickButton(driverController, Controller_Constants.TriangleButton)
     .onTrue(led_Commands.coneLEDS());

     new JoystickButton(driverController, Controller_Constants.CrossButton)
     .onTrue(punch_Commands.Punch())
     .onTrue(led_Commands.emote())
     .onFalse(led_Commands.emote());

     /*
     new JoystickButton(driverController, Controller_Constants.CircleButton)
     .onTrue(led_Commands.hotPink());

     new JoystickButton(driverController, Controller_Constants.LBPort)
     .onTrue(led_Commands.rally())
     .onFalse(led_Commands.rally());

     new JoystickButton(driverController, Controller_Constants.RBPort)
     .onTrue(led_Commands.gold())
     .onFalse(led_Commands.gold());*/
  }  

  /**
  public void runAuto() {
    if(Shuffleboard.autoChoice.getSelected().equals("TimedMobility"))
    {
      
    autoManager.timedMobility.schedule();
    }
    else if(Shuffleboard.autoChoice.getSelected().equals("TimedEngage"))
    {
      autoManager.timedEngage.schedule();

    }
    else if(Shuffleboard.autoChoice.getSelected().equals("TimedMobilityEngage"))
    {
      autoManager.timedMobilityEngage.schedule();

    }
    else
    {
      autoManager.runAuto(shuffleboard.getAuto());
    }
  }*/

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
