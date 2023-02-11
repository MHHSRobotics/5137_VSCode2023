// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;

public class DriveBase_Subsystem extends SubsystemBase {
  //left motors
  public static WPI_TalonSRX leftFrontTalon;
  public static WPI_TalonSRX leftBackTalon;
  public static MotorControllerGroup leftDrive;
  
  //right motors
  public static WPI_TalonSRX rightFrontTalon;
  public static WPI_TalonSRX rightBackTalon;
  public static MotorControllerGroup rightDrive;


  //DriveTrain
  public static DifferentialDrive jMoney_Drive;

  //Controller
  Joystick controller;

  //gyros
  ADXRS450_Gyro horizontalGyro;

  public DriveBase_Subsystem() {
    //left motors
    leftFrontTalon = new WPI_TalonSRX(Constants.leftFrontTalonPort);
    leftBackTalon = new WPI_TalonSRX(Constants.leftBackTalonPort);
    leftDrive = new MotorControllerGroup(leftFrontTalon, leftBackTalon);

    //right motors
    rightFrontTalon = new WPI_TalonSRX(Constants.rightFrontTalonPort);
    rightBackTalon = new WPI_TalonSRX(Constants.leftBackTalonPort);
    rightDrive = new MotorControllerGroup(rightFrontTalon, rightBackTalon);

    //DriveTrain
    jMoney_Drive = new DifferentialDrive(leftDrive, rightDrive);

    //Controller
    controller = Robot.driverController;

    //Gyros
    horizontalGyro = new ADXRS450_Gyro();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (controller != null) {
      arcadeDrive(controller);
    }
  }

  public void arcadeDrive(Joystick controller) {
    //Gets controller values
    double speed = controller.getRawAxis(Constants.g_LYStickAxisPort);
    double rotate = controller.getRawAxis(Constants.d_RXStickAxisPort);
    speed = adjust(speed);
    rotate = adjust(rotate);
    jMoney_Drive.curvatureDrive(-speed/Constants.driveSensitivity, -rotate/Constants.turnSensitivity, true);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double axis) {
    if (Math.abs(axis)<Constants.errormargin) {return 0.0;}
    if (Math.abs(axis)>(1-Constants.errormargin)) {return (Math.abs(axis)/axis);}
    return axis;
  }

  public void drive(double speed, double rotate) {
    jMoney_Drive.curvatureDrive(-speed/Constants.driveSensitivity, -rotate/Constants.turnSensitivity, true);
  }
}