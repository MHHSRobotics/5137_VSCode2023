// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveBaseSubsystem extends SubsystemBase {
  //left motors
  public static WPI_TalonSRX leftTalon;
  public static WPI_VictorSPX leftFrontVic;
  public static WPI_VictorSPX leftBackVic;
  public static MotorControllerGroup leftDrive;
  
  //right motors
  public static WPI_TalonSRX rightTalon;
  public static WPI_VictorSPX rightFrontVic;
  public static WPI_VictorSPX rightBackVic;
  public static MotorControllerGroup rightDrive;

  //DriveTrain
  DifferentialDrive testDrive;

  //Systems

  public static double rightDist;
  public static double leftDist;


  //Gyro 
  public static ADXRS450_Gyro gyro;

  //Controller
  Joystick controller;

  //Joystick Ports
  int LYStickAxisPort;
  int RXStickAxisPort;



  public DriveBaseSubsystem() {

    
    
    
    //Gyro
    gyro = new ADXRS450_Gyro();
    gyro.calibrate();


    //left motors
    leftTalon = new WPI_TalonSRX(Constants.leftTalonPort);
    leftFrontVic = new WPI_VictorSPX(Constants.leftFrontVicPort);
    leftBackVic = new WPI_VictorSPX(Constants.leftBackVicPort);
    leftDrive = new MotorControllerGroup(leftTalon, leftFrontVic, leftBackVic);

    //right motors
    rightTalon = new WPI_TalonSRX(Constants.rightTalonPort);
    rightFrontVic = new WPI_VictorSPX(Constants.rightFrontVicPort);
    rightBackVic = new WPI_VictorSPX(Constants.rightBackVicPort);
    rightDrive = new MotorControllerGroup(rightTalon, rightFrontVic, rightBackVic);


    leftFrontVic.setInverted(true);
    //DriveTrain
    testDrive = new DifferentialDrive(leftDrive, rightDrive);
    leftDrive.setInverted(true);
    
    //Controller
    controller = new Joystick(Constants.controllerPort);

    LYStickAxisPort = Constants.XBOX_LYStickAxisPort;
    RXStickAxisPort = Constants.XBOX_RXStickAxisPort;

    /*
    //Controller Type (Not required just for ease)
    switch (Constants.controllerType) {
      case ("xbox"):
        LYStickAxisPort = Constants.XBOX_LYStickAxisPort;
        RXStickAxisPort = Constants.XBOX_RXStickAxisPort;
        System.out.println("XBox Controller");
        return;
      case ("ps4"):
        LYStickAxisPort = Constants.PS4_LYStickAxisPort;
        RXStickAxisPort = Constants.PS4_RXStickAxisPort;
        System.out.println("PS4 Controller");
        return;
      default:
        LYStickAxisPort = Constants.XBOX_LYStickAxisPort;
        RXStickAxisPort = Constants.XBOX_RXStickAxisPort;
        System.out.println("Default Controller");
    }
    */
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeDrive(controller);
    //System.out.println(gyro.getAngle());
    
    //Encoders
    rightDist = DriveBaseSubsystem.rightTalon.getSelectedSensorPosition() * Constants.distancePerPulse;
    leftDist = -DriveBaseSubsystem.leftTalon.getSelectedSensorPosition() * Constants.distancePerPulse;
  }
   

  public void arcadeDrive(Joystick controller) {
    //Gets controller values
    double speed = controller.getRawAxis(LYStickAxisPort);
    double rotate = controller.getRawAxis(RXStickAxisPort);
    speed = adjust(speed);
    rotate = adjust(rotate);
    testDrive.curvatureDrive(speed/Constants.driveSensitivity, rotate/Constants.turnSensitivity, true);
  }

  public void driveStraight(double speed){
    testDrive.curvatureDrive(speed, 0, false);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double x) {
    if (Math.abs(x)<Constants.errormargin) {return 0.0;}
    if (Math.abs(x)>(1-Constants.errormargin)) {return (Math.abs(x)/x);}
    return x;
  }
}
