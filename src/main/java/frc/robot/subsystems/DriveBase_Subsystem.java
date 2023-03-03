// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveBase_Subsystem extends SubsystemBase {
  //left motors
  public static WPI_TalonFX leftFrontTalon;
  public static WPI_TalonFX leftBackTalon;
  public static MotorControllerGroup leftDrive;
  
  //right motors
  public static WPI_TalonFX rightFrontTalon;
  public static WPI_TalonFX rightBackTalon;
  public static MotorControllerGroup rightDrive;

  //DriveTrain
  DifferentialDrive jMoney_Drive;

  //Position Estimator
  private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, new Rotation2d(0),Constants.initialLeftDistance ,Constants.initialRightDistance, new Pose2d());

  //Controller
  Joystick controller;
  PIDController distanceController;
  PIDController rotationController;
  PIDController balanceController;
  public SimpleMotorFeedforward voltPID;

  //gyros
  public static AHRS gyro;

  

  //Paths
  public ArrayList<PathPlannerTrajectory> score_mobility_chargeEngage;
  public ArrayList<PathPlannerTrajectory> score_mobility_intake_score;
  public ArrayList<PathPlannerTrajectory> score_chargeEngage;
  public ArrayList<PathPlannerTrajectory> score_mobility_straightChargeEngage;
  public ArrayList<PathPlannerTrajectory> Goal_Path;

  //AutoBuilder

  //RobotContainer

  //rate limiter
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(1.2);
  private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(4);



  public DriveBase_Subsystem() {

    score_mobility_chargeEngage = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("score_mobility_chargeEngage", new PathConstraints(4, 3));
    score_mobility_intake_score = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("score_mobility_intake_score", new PathConstraints(4, 3));
    score_chargeEngage =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("score_chargeEngage", new PathConstraints(4, 3));
    score_mobility_straightChargeEngage =  (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("score_mobility_straightChargeEngage", new PathConstraints(2, 1));
    Goal_Path = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Goal_Path", new PathConstraints(4, 3));

    //Maps for the path groups
    leftFrontTalon = new WPI_TalonFX(Constants.leftFrontTalonPort);
    leftBackTalon = new WPI_TalonFX(Constants.leftBackTalonPort);
    leftDrive = new MotorControllerGroup(leftFrontTalon, leftBackTalon);
    //right motors
    rightFrontTalon = new WPI_TalonFX(Constants.rightFrontTalonPort);
    rightBackTalon = new WPI_TalonFX(Constants.rightBackTalonPort);
    rightDrive = new MotorControllerGroup(rightFrontTalon, rightBackTalon);
    rightDrive.setInverted(true);

    
    //Gyros
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.calibrate();

    //position estimator 
    poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, new Rotation2d(gyro.getRoll()),Constants.initialLeftDistance ,Constants.initialRightDistance, new Pose2d());


    //Encoders 
    leftFrontTalon.setSelectedSensorPosition(0);
    rightFrontTalon.setSelectedSensorPosition(0);

    //DriveTrain
    jMoney_Drive = new DifferentialDrive(leftDrive, rightDrive);
    jMoney_Drive.setMaxOutput(.7);
    

    //PID
    distanceController = new PIDController(Constants.dKP,Constants.dKI, Constants.dKD);
    rotationController = new PIDController(Constants.rKP,Constants.rKI, Constants.rKD);
    balanceController = new PIDController(Constants.bKP,Constants.bKI, Constants.bKD);
    voltPID = new SimpleMotorFeedforward(Constants.dKS, Constants.dKV, Constants.dKA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (controller != null) {
      arcadeDrive(controller);
    }
    //Updates the position with gyro and encoder periodcally 
    updatePoseEstimator();

    //For Testing
    //System.out.println(getPose());
    
    


    //System.out.println("Pitch (Vertical)" + gyro.getPitch());


  }


  //A consumer(method that takes a value) used in the auto paths / autoBuilder that drives the robot using a left and right speed
  public void drive(double leftSpeed, double rightSpeed)
  {
   System.out.print("left speed" + leftSpeed);
   System.out.print("right speed" + rightSpeed);
    //jMoney_Drive.tankDrive(leftSpeed, rightSpeed);
  }

  //Returns wheel speeds of motors
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    double leftSpeed = leftFrontTalon.getSelectedSensorVelocity()*Constants.distancePerPulse_TalonFX*10; //Speed = sensor count per 100 ms * distance per count * 10 (converts 100 ms to s)
    double rightSpeed = rightFrontTalon.getSelectedSensorVelocity()*Constants.distancePerPulse_TalonFX*10;
    
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  //Sets the volts of each motor 
  public void setVolts(double leftVolts, double rightVolts)
  {
    
    leftVolts *= .2;
    rightVolts *= .2;
    leftVolts -= 0.1*leftVolts;

    leftDrive.setVoltage(-leftVolts);
    rightDrive.setVoltage(-rightVolts);
    System.out.println(getWheelSpeeds());
    System.out.println(leftVolts);
  }


  //Used by the bot to drive -- calls upon adjust method to reduce error. Is used by the DefaultDrive command to drive in TeleOp
  public void arcadeDrive(Joystick controller) {
    //Gets controller values
    double speed = controller.getRawAxis(1);
    double rotate = controller.getRawAxis(4);

    speed = adjust(speed);
    rotate = adjust(rotate);
    speed = rateLimiter.calculate(speed);
    rotate = rotateLimiter.calculate(rotate);
    //rotate = rateLimiter.calculate(rotate);
  //System.out.println("speed:" + speed);
  //System.out.println("rotate" + rotate);
  if(speed!=0.0){
    rotate -= 0.4*speed;
  }
  //System.out.println("speed:" + speed);
  //System.out.println("rotate" + rotate);  
    jMoney_Drive.curvatureDrive(speed/Constants.driveSensitivity, rotate/Constants.turnSensitivity , true);

  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double axis) {
    if (Math.abs(axis)<Constants.errormargin) {return 0.0;}
    if (Math.abs(axis)>(1-Constants.errormargin)) {return (Math.abs(axis)/axis);}
    return axis;
  }

  //Automatically Balances on charge station using gyro measurements
  public double autoBalance()
  {
    double forwardSpeed = balanceController.calculate(gyro.getPitch(), 0); //Calculates forward speed using PID
    jMoney_Drive.curvatureDrive(forwardSpeed, 0, false);; //Sets the drivetraub to drive forward/backwards using PID speed
    return forwardSpeed;
  }

  //Drives towards and rotates towards a given position based on distance and yaw using PIDs
  public double autoDrive(Pose2d targetPose) 
  {
    double forwardSpeed = distanceController.calculate(PhotonUtils.getDistanceToPose(getPose(), targetPose), 0); //Calculates forward speed using PID
    double rotateSpeed =  -rotationController.calculate(PhotonUtils.getYawToPose(getPose(), targetPose).getDegrees(), 0.0);
    jMoney_Drive.curvatureDrive(forwardSpeed, rotateSpeed, false);; //Sets the drivetraub to drive forward/backwards using PID speed
    return forwardSpeed;
  }

  //Rotate towards a given pose based on yaw using a PID
  public double autoRotate(Pose2d targetPose)
  {
    double rotateSpeed =  -rotationController.calculate(PhotonUtils.getYawToPose(getPose(), targetPose).getDegrees(), 0.0);
    jMoney_Drive.curvatureDrive(0, rotateSpeed, true);
    return rotateSpeed;
  }
  
  //Returns the current global pose estimate of robot
  public Pose2d getPose()
  {
    return poseEstimator.getEstimatedPosition(); 
    
  }

  //Updates the pose estimator with current encoder values and gyro readings
  public void updatePoseEstimator(){
    //Make sure timer delay is added if needed, could need because of motor delays from inversion
    double leftFrontEncoder = leftFrontTalon.getSelectedSensorPosition() * Constants.distancePerPulse_TalonFX;
    double rightFrontEncoder = rightFrontTalon.getSelectedSensorPosition() * Constants.distancePerPulse_TalonFX;
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d((double)gyro.getRoll()), leftFrontEncoder, rightFrontEncoder); //ad gyro value
  } 
   
  //Used by AddVisionMeasurement command to add a location to pose estimator based on april tag system
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds)
  {
  poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }


  //Resets global pose estimator based on a position parameter, gyro and encoder have no effect
  public void resetPose(Pose2d pose)
  {
    poseEstimator.resetPosition(new Rotation2d(gyro.getRoll()), 0, 0, pose);
  }
  }




