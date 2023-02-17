// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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

  //Position Estimator
  private DifferentialDrivePoseEstimator poseEstimator;
  //Controller
  Joystick controller;
  PIDController distanceController;
  PIDController rotationController;
  PIDController balanceController;

  //gyros
  public static PigeonIMU gyro;

  public DriveBase_Subsystem() {
    //left motors
    leftFrontTalon = new WPI_TalonSRX(Constants.leftFrontTalonPort);
    leftBackTalon = new WPI_TalonSRX(Constants.leftBackTalonPort);
    leftDrive = new MotorControllerGroup(leftFrontTalon, leftBackTalon);

    //right motors
    rightFrontTalon = new WPI_TalonSRX(Constants.rightFrontTalonPort);
    rightBackTalon = new WPI_TalonSRX(Constants.leftBackTalonPort);
    rightDrive = new MotorControllerGroup(rightFrontTalon, rightBackTalon);

    //Gyros
    gyro = new PigeonIMU(0);

    //position estimator 
    poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, new Rotation2d(gyro.getRoll()),Constants.initialLeftDistance ,Constants.initialRightDistance, new Pose2d());


    //Encoders 
    leftFrontTalon.setSelectedSensorPosition(0);

    //DriveTrain
    jMoney_Drive = new DifferentialDrive(leftDrive, rightDrive);

    //Controller
    controller = Robot.driverController;

    

    //PID
    distanceController = new PIDController(Constants.dKP,Constants.dKI, Constants.dKD);
    rotationController = new PIDController(Constants.rKP,Constants.rKI, Constants.rKD);
    balanceController = new PIDController(Constants.bKP,Constants.bKI, Constants.bKD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (controller != null) {
      arcadeDrive(controller);
    }
    updatePoseEstimator();
  }

  public void arcadeDrive(Joystick controller) {
    //Gets controller values
    double speed = controller.getRawAxis(Constants.g_LYStickAxisPort);
    double rotate = controller.getRawAxis(Constants.d_RXStickAxisPort);
    speed = adjust(speed);
    rotate = adjust(rotate);
    jMoney_Drive.curvatureDrive(-speed/Constants.driveSensitivity, -rotate/Constants.turnSensitivity, true);
  }

  public void drive(double speed, double rotate) {
    jMoney_Drive.curvatureDrive(speed, rotate, true);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double axis) {
    if (Math.abs(axis)<Constants.errormargin) {return 0.0;}
    if (Math.abs(axis)>(1-Constants.errormargin)) {return (Math.abs(axis)/axis);}
    return axis;
  }

  public double autoBalance()
  {
    double forwardSpeed = balanceController.calculate(gyro.getPitch(), 0); //Calculates forward speed using PID
    jMoney_Drive.curvatureDrive(forwardSpeed, 0, false);; //Sets the drivetraub to drive forward/backwards using PID speed
    return forwardSpeed;
    
  }



  public double autoDrive(Pose2d targetPose) 
  {
    double forwardSpeed = distanceController.calculate(PhotonUtils.getDistanceToPose(getPose(), targetPose), 0); //Calculates forward speed using PID
    double rotateSpeed =  -rotationController.calculate(PhotonUtils.getYawToPose(getPose(), targetPose).getDegrees(), 0.0);
    jMoney_Drive.curvatureDrive(forwardSpeed, rotateSpeed, false);; //Sets the drivetraub to drive forward/backwards using PID speed
    return forwardSpeed;
  }

  public double autoRotate(Pose2d targetPose)
  {
    double rotateSpeed =  -rotationController.calculate(PhotonUtils.getYawToPose(getPose(), targetPose).getDegrees(), 0.0);
    jMoney_Drive.curvatureDrive(0, rotateSpeed, true);
    return rotateSpeed;
  }
  
  public Pose2d getPose()
  {
    return poseEstimator.getEstimatedPosition(); 
    
  }

  public void updatePoseEstimator(){
    //Make sure timer delay is added if needed, could need because of motor delays from inversion
    double leftFrontEncoder = leftFrontTalon.getSelectedSensorPosition() * Constants.distancePerPulse_TalonSRX;
    double rightFrontEncoder = rightFrontTalon.getSelectedSensorPosition() * Constants.distancePerPulse_TalonSRX;
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(gyro.getRoll()), leftFrontEncoder, rightFrontEncoder);
  } 
   
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds)
  {
  poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  public void resetPose(Pose2d pose)
  {
    poseEstimator.resetPosition(new Rotation2d(gyro.getRoll()), 0, 0, pose);
  }
  
  

  }




