// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import java.util.ArrayList;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Arm_Subsystem;
//Subsystems
import frc.robot.subsystems.*;

//Commands
import frc.robot.commands.Intake_Commands.*;
import frc.robot.commands.Vision_Commands.AddVisionMeasurement;
import frc.robot.commands.Compressor_Commands.*;
import frc.robot.commands.Drive_Commands.AutoBalance;
import frc.robot.commands.Drive_Commands.AutoDrive;
import frc.robot.commands.Drive_Commands.AutoRotate;
import frc.robot.commands.Clamp_Commands.*;
import frc.robot.commands.Arm_Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  public static DriveBase_Subsystem driveBase_Subsystem;
  public static Intake_Subsystem intake_Subsystem;
  public static Pneumatics_Subsystem pneumatics_Subsystem;
  public static Clamp_Subsystem clamp_Subsystem;
  public static Arm_Subsystem arm_Subsystem;
  public static SmartDashboard_Subsystem smartDashboard_Subsystem;
  public Vision_Subsystem vision_Subsystem;

  //Controllers
  public static Joystick driverController;
  public static Joystick assistController;
  public static String controllerType;

  //Triggers
  public static Trigger driver_LTrigger;
  public static Trigger driver_RTrigger;
  public static Trigger assist_LTrigger;
  public static Trigger assist_RTrigger;

  //Buttons
  public static JoystickButton driver_StartButton; //Maybe switch these to assist
  public static JoystickButton driver_BackButton;
  public static JoystickButton driver_XButton;
  public static JoystickButton driver_AButton;
  public static JoystickButton driver_BButton;
  public static JoystickButton assist_XButton;
  public static JoystickButton assist_AButton;
  public static JoystickButton assist_YButton;
  public static JoystickButton assist_BButton;
  public static POVButton DownDPad;
  public static POVButton UpDPad;

  public RobotContainer() {
    //Subsystems
    pneumatics_Subsystem = new Pneumatics_Subsystem();
    driveBase_Subsystem = new DriveBase_Subsystem();
    intake_Subsystem = new Intake_Subsystem(); 
    clamp_Subsystem = new Clamp_Subsystem();
    arm_Subsystem = new Arm_Subsystem();
    smartDashboard_Subsystem = new SmartDashboard_Subsystem();

    vision_Subsystem = new Vision_Subsystem();
    //Controllers
    driverController = Robot.driverController;
    assistController = Robot.assistController;
    vision_Subsystem.setDefaultCommand(new AddVisionMeasurement(driveBase_Subsystem, vision_Subsystem));
  }

  public static void configureBindings() {

    //Align Testing 
    driver_XButton = new JoystickButton(driverController, Constants.d_XSquaredPort);
    driver_XButton.whileTrue(new AutoRotate(driveBase_Subsystem, Constants.pose2b));

    driver_BButton = new JoystickButton(driverController, Constants.d_BirclePort);
    driver_BButton.whileTrue(new AutoDrive(driveBase_Subsystem, Constants.pose2b));
    
    driver_AButton = new JoystickButton(driverController, Constants.d_AxePort);
    driver_AButton.whileTrue(new AutoBalance(driveBase_Subsystem));

    //Intake 
    driver_LTrigger = new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_LTriggerPort, Constants.d_RTriggerPort));
    driver_LTrigger.whileTrue(new IntakeOnReverse());
    driver_LTrigger.onFalse(new IntakeOff());

    driver_RTrigger = new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_RTriggerPort, Constants.d_LTriggerPort));
    driver_RTrigger.whileTrue(new IntakeOn());
    driver_RTrigger.onFalse(new IntakeOff());

    //Compressor 
    driver_StartButton = new JoystickButton(driverController, Constants.d_StoptionsPort);
    driver_StartButton.onTrue(new CompressorOn());

    driver_BackButton = new JoystickButton(driverController, Constants.d_ShackPort);
    driver_BackButton.onTrue(new CompressorOff());

    //Arm
    assist_XButton = new JoystickButton(assistController, Constants.a_XSquaredPort);
    assist_XButton.onTrue(new TopConePreset());

    assist_AButton = new JoystickButton(assistController, Constants.a_AxePort);
    assist_AButton.onTrue(new MidConePreset());

    assist_YButton = new JoystickButton(assistController, Constants.g_Yangle);
    assist_YButton.onTrue(new TopCubePreset());

    assist_BButton = new JoystickButton(assistController, Constants.a_BirclePort);
    assist_BButton.onTrue(new MidCubePreset());

    UpDPad = new POVButton(assistController, Constants.UpDPad);
    UpDPad.onTrue(new ArmResetToIntake());

    DownDPad = new POVButton(assistController, Constants.DownDPad);
    DownDPad.onTrue(new HybridPreset());

    //Clamp
    assist_LTrigger = new Trigger(Supplier.createBooleanSupplier(assistController, Constants.a_LTriggerPort, Constants.a_RTriggerPort));
    assist_LTrigger.onTrue(new ClampCube());
    assist_LTrigger.onFalse(new ClampOpen());

    assist_RTrigger = new Trigger(Supplier.createBooleanSupplier(assistController, Constants.a_RTriggerPort, Constants.a_LTriggerPort));
    assist_RTrigger.onTrue(new ClampCone());
    assist_RTrigger.onFalse(new ClampOpen());
  }

  public static Command getAutoCommand(String autoChose){
    if (autoChose == "score_chargeEngage"){
      return driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_chargeEngage);
    } 
    else if (autoChose == "score_mobility_chargeEngage"){
      return driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_chargeEngage);
    } 
    else if (autoChose == "score_mobility_intake_score"){
      return driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_intake_score);
    } 
    else if (autoChose == "Goal_Path"){
      return driveBase_Subsystem.autoBuilder.fullAuto(driveBase_Subsystem.Goal_Path);
    }
    return null;
  }
  
}
