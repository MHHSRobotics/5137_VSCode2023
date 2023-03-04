// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm_Subsystem;
//Subsystems
import frc.robot.subsystems.*;

//Commands
import frc.robot.commands.Vision_Commands.AddVisionMeasurement;
import frc.robot.commands.Compressor_Commands.*;
import frc.robot.commands.Drive_Commands.AutoBalance;
import frc.robot.commands.Drive_Commands.AutoDrive;
import frc.robot.commands.Drive_Commands.AutoRotate;
import frc.robot.commands.Drive_Commands.DefaultDrive;
import frc.robot.commands.Clamp_Commands.*;
import frc.robot.commands.Arm_Commands.*;
import frc.robot.commands.Intake_Commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //Subsytems 
  private final  Pneumatics_Subsystem pneumatics_Subsystem = new Pneumatics_Subsystem();
  private final  DriveBase_Subsystem driveBase_Subsystem = new DriveBase_Subsystem();
  private final  Intake_Subystem intake_Subystem = new Intake_Subystem(); 
  private final  Clamp_Subsystem clamp_Subsystem = new Clamp_Subsystem();
  private final  Arm_Subsystem arm_Subsystem = new Arm_Subsystem();
  private final  Addressable_LEDS_Subsystem addressable_LEDS_Subsystem = new Addressable_LEDS_Subsystem();
  //private final  Arm_Subsystem1 arm_Subsystem1 = new Arm_Subsystem1();

  private final  Vision_Subsystem vision_Subsystem = new Vision_Subsystem();


  //Controllers
  private final Joystick driverController  = new Joystick(Constants.driverControllerPort);
  private final Joystick assistController = new Joystick(Constants.assistControllerPort);

  //Variables for running auto
  private final SendableChooser<Command> autoChooser = new SendableChooser<>(); //Chooser for auto mode
  private final RamseteAutoBuilder autoBuilder; //Allows auto to drive a path
  private final HashMap<String, Command> eventMap = new HashMap<>(); //Maps out events during autoPath

  //Used by Supplier statically to decide what the control bindings should be 
  public static SendableChooser<String> driverControlChooser = new SendableChooser<>();
  public static SendableChooser<String> assistControlChooser = new SendableChooser<>();
  public static SendableChooser<String> ledChooser = new SendableChooser<>();

  

  public RobotContainer() {
    //Smart Dashboard Options for LED Modes
    ledChooser.setDefaultOption("Team Colors", "teamColors");
    ledChooser.addOption("Cardinal", "cardinal");
    SmartDashboard.putData("LED MODE", ledChooser);

    //SmartDashboard values for choosing controller type, twill be overwritten by Joseph's shuffleboard
    driverControlChooser.setDefaultOption("XBOX", "xbox");
    driverControlChooser.addOption("PLAY_STATION", "ps4");

    assistControlChooser.setDefaultOption("XBOX", "xbox");
    assistControlChooser.addOption("PLAY_STATION", "ps4");
    
    SmartDashboard.putData("Driver Controller Type", driverControlChooser);
    SmartDashboard.putData("Assist Controller Type", assistControlChooser);

    //Sets the default command for drivebase to drive using controller
    driveBase_Subsystem.setDefaultCommand(new DefaultDrive(driveBase_Subsystem, driverController));
    //Sets the defasult command to run continously for the vision subsystem
    vision_Subsystem.setDefaultCommand(new AddVisionMeasurement(driveBase_Subsystem, vision_Subsystem));
    
    /* 
    //Command group for scoring, relies on the isFinished of each command
    SequentialCommandGroup score = new SequentialCommandGroup(new IntakeExtend(intake_Subystem),new ClampCone(clamp_Subsystem), new TopConePreset(arm_Subsystem, intake_Subystem) ,  new ClampOpen(clamp_Subsystem), new PrintCommand("Score Finished"));
    
    //Adds commands to be used at event markers during auto path. Used as a parameter in autoBuilder
    eventMap.put("Intake1", new IntakeOn(intake_Subystem));
    eventMap.put("Score1", score);
    eventMap.put("Balance1", new AutoBalance(driveBase_Subsystem));
    */
    //initializes the auto builder which runs an the auto paths
    autoBuilder = new RamseteAutoBuilder(
      driveBase_Subsystem::getPose, // Pose2d supplier method from drivetrain
      driveBase_Subsystem::resetPose, // Pose2d consume method used to reset odometry at the beginning of auto
      new RamseteController(),
      Constants.trackWidth, //Kinematics for our drivebase
      driveBase_Subsystem.voltPID,
      driveBase_Subsystem::getWheelSpeeds,
      Constants.drivePIDConstants,
      driveBase_Subsystem::setVolts,
      eventMap, //Event map that maps out commands to specific keywords in autoPath markers
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      driveBase_Subsystem); // The drive subsystem. Used to properly set the requirements of path following commands

     //Adds auto options to SmartDashboard, will be replaced by Shuffleboard
     autoChooser.setDefaultOption("score_chargeEngage", autoBuilder.fullAuto(driveBase_Subsystem.score_chargeEngage)); //.fullAuto() method runs the autoBuilder using a specific path
     autoChooser.addOption("score_mobility_chargeEngage", autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_chargeEngage));
     autoChooser.addOption("score_mobility_intake_score", autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_intake_score));
     autoChooser.setDefaultOption("score_mobility_straightChargeEngage", autoBuilder.fullAuto(driveBase_Subsystem.score_mobility_straightChargeEngage)); //.fullAuto() method runs the autoBuilder using a specific path
     autoChooser.addOption("Goal_Path", autoBuilder.fullAuto(driveBase_Subsystem.Goal_Path));
     SmartDashboard.putData("Auto Modes", autoChooser);
  }
  
  public void configureBindings() {

    //Automated Commands
    new JoystickButton(driverController, 1)
  .whileTrue(new InstantCommand(() -> arm_Subsystem.setRotation(10) , arm_Subsystem)); //AutoRotate


    
    new JoystickButton(driverController, Constants.d_BirclePort)  
    .whileTrue(new AutoDrive(driveBase_Subsystem, Constants.pose2b)); //AutoDrive
    
    new JoystickButton(driverController, Constants.d_AxePort) //A Button
    .whileTrue(new AutoBalance(driveBase_Subsystem)); //Balancing

    //Intake Commands
    /* 
    new JoystickButton(driverController, 1) 
    .whileTrue(new IntakeOnReverse(intake_Subystem)) //Intake running in reverse
    .whileFalse(new IntakeOff(intake_Subystem)); 
     
    new JoystickButton(driverController, Constants.d_RTriggerPort) 
    .whileTrue(new IntakeOn(intake_Subystem)) //Intake running
    .whileFalse(new IntakeOff(intake_Subystem));
    */
    

    new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_RTriggerPort, Constants.d_LTriggerPort))
    .whileTrue(new IntakeOn(intake_Subystem)) //Intake running
    .onFalse(new IntakeOff(intake_Subystem));

    new Trigger(Supplier.createBooleanSupplier(driverController, Constants.d_LTriggerPort, Constants.d_RTriggerPort))
    .whileTrue(new IntakeOnReverse(intake_Subystem)) //Intake running
    .onFalse(new IntakeOff(intake_Subystem));
    
    //Compressor Commands
    new JoystickButton(driverController, Constants.d_StoptionsPort)
    .onTrue(new CompressorOn(pneumatics_Subsystem)); //Compressor On

    new JoystickButton(driverController, Constants.d_ShackPort)
    .onTrue(new CompressorOff(pneumatics_Subsystem)); //Compressor Off

    //Arm Commands
    new JoystickButton(assistController, Constants.a_XSquaredPort)
    .onTrue(new TopConePreset(arm_Subsystem, intake_Subystem)); //Top Cone

    new JoystickButton(assistController, Constants.a_AxePort)
    .onTrue(new MidConePreset(arm_Subsystem, intake_Subystem)); //Mid Cone

    new JoystickButton(assistController, Constants.g_Yangle)
    .onTrue(new TopCubePreset(arm_Subsystem, intake_Subystem)); //Top Cube

    new JoystickButton(assistController, Constants.a_BirclePort)
    .onTrue(new MidCubePreset(arm_Subsystem)); //Mid Cube

    new POVButton(assistController, Constants.UpDPad)
    .onTrue(new ArmResetToIntake(arm_Subsystem, intake_Subystem)); //Reset to Intake Position

    new POVButton(assistController, Constants.DownDPad)
    .onTrue(new HybridPreset(arm_Subsystem, intake_Subystem)); //Floor-Hybrid Positon 

    //Clamp Commands
    new JoystickButton(assistController, Constants.a_RTriggerPort)
    .whileTrue(new ClampCube(clamp_Subsystem)) //Clamp Cube
    .whileFalse(new ClampOpen(clamp_Subsystem));
    
    new JoystickButton(assistController, Constants.a_RTriggerPort)
    .whileTrue(new ClampCone(clamp_Subsystem)) //Clamp Cone
    .whileFalse(new ClampOpen(clamp_Subsystem));

  }

  //Returns the current selected auto command based on sendable. If none is selected goes to default command. If no default returns null. 
  public Command getAutoCommand(){
    return autoChooser.getSelected();
  }

  //Method to call LED MOde with specfic colors 
  public void setLedMode()
  {
    if(ledChooser.getSelected().equals("teamColors"))
    {
      addressable_LEDS_Subsystem.alternateColors(Constants.cardinal, Constants.gold);
    }
    else if(ledChooser.getSelected().equals("cardinal"))
    {
      addressable_LEDS_Subsystem.solidColor(Constants.cardinal);
    }
  }

  public void moveColorsForward() {
    addressable_LEDS_Subsystem.moveForward(Constants.cardinal, Constants.gold);
  }

  public void moveColorsBackward() {
    addressable_LEDS_Subsystem.moveBackward(Constants.cardinal, Constants.gold);
  }

  public void solidCardinal() {
    addressable_LEDS_Subsystem.solidColor(Constants.cardinal);
  }

  public void solidGold() {
    addressable_LEDS_Subsystem.solidColor(Constants.gold);
  }

  public void gradiant() {
    addressable_LEDS_Subsystem.gradiant();
  }

  public void cardinalgold() {
    addressable_LEDS_Subsystem.pulsingGradiant(Constants.gold, Constants.cardinal, 1);
  }

  public void s(){
    addressable_LEDS_Subsystem.s(Constants.cardinal, Constants.gold, 0);
  }

  public void rainbow() {
    addressable_LEDS_Subsystem.rainbow();
  }
}
