// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveBase_Subsystem extends SubsystemBase {
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

  //Controller
  Joystick controller;

  //Joystick Ports
  int LYStickAxisPort;
  int RXStickAxisPort;

  public DriveBase_Subsystem() {
    //left motors
    leftTalon = new WPI_TalonSRX(Constants.leftTalonPort);
    leftFrontVic = new WPI_VictorSPX(Constants.leftFrontVicPort);
    leftBackVic = new WPI_VictorSPX(Constants.leftBackVicPort);
    leftDrive = new MotorControllerGroup(leftTalon, leftFrontVic, leftBackVic);

    leftFrontVic.setInverted(true);

    //right motors
    rightTalon = new WPI_TalonSRX(Constants.rightTalonPort);
    rightFrontVic = new WPI_VictorSPX(Constants.rightFrontVicPort);
    rightBackVic = new WPI_VictorSPX(Constants.rightBackVicPort);
    rightDrive = new MotorControllerGroup(rightTalon, rightFrontVic, rightBackVic);

    //DriveTrain
    testDrive = new DifferentialDrive(leftDrive, rightDrive);

    //Controller
    controller = new Joystick(Constants.controllerPort);

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeDrive(controller);
  }

  public void arcadeDrive(Joystick controller) {
    //Gets controller values
    double speed = controller.getRawAxis(LYStickAxisPort);
    double rotate = controller.getRawAxis(RXStickAxisPort);
    speed = adjust(speed);
    rotate = adjust(rotate);
    testDrive.curvatureDrive(speed/Constants.driveSensitivity, rotate/Constants.turnSensitivity, true);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double x) {
    if (Math.abs(x)<Constants.errormargin) {return 0.0;}
    if (Math.abs(x)>(1-Constants.errormargin)) {return (Math.abs(x)/x);}
    return x;
  }
}
