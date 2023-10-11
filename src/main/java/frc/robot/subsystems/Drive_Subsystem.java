// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.EnumKeySerializer;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;
import frc.robot.constants.Controller_Constants;

public class Drive_Subsystem extends SubsystemBase {
  //left motors
  public static WPI_TalonFX leftFrontTalon;
  public static WPI_TalonFX leftBackTalon;
  public static MotorControllerGroup leftDrive;
  
  //right motors
  public static WPI_TalonFX rightFrontTalon;
  public static WPI_TalonFX rightBackTalon;
  public static MotorControllerGroup rightDrive;

  //DriveTrain
  public static DifferentialDrive jMoneyDrive;

  //Position Estimator
  public DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Drive_Constants.trackWidth, new Rotation2d(0), Drive_Constants.initialLeftDistance, Drive_Constants.initialRightDistance, new Pose2d());

  //Controller
  Joystick controller;
  PIDController distanceController;
  PIDController rotationController;
  PIDController balanceController;
  public SimpleMotorFeedforward voltPID;

  //gyros
  public static AHRS gyro;

  //Commands 
  public BooleanSupplier balanceIsFinished;
  public Consumer<Boolean> balanceEndCommand;
  public Consumer<Boolean> tagDriveEndCommands;  

  //Timer
  private Timer timer = new Timer();

  //rate limiter
  private final SlewRateLimiter rateLimiter = new SlewRateLimiter(13); //1.2
  private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(10); //4



  public Drive_Subsystem(Joystick m_controller) {

    
    //Maps for the path groups
    leftFrontTalon = new WPI_TalonFX(Drive_Constants.leftFrontTalonPort);
    leftBackTalon = new WPI_TalonFX(Drive_Constants.leftBackTalonPort);
    leftDrive = new MotorControllerGroup(leftFrontTalon, leftBackTalon);
    leftDrive.setInverted(true);

    //right motors
    rightFrontTalon = new WPI_TalonFX(Drive_Constants.rightFrontPort);
    rightBackTalon = new WPI_TalonFX(Drive_Constants.rightBackPort);
    rightDrive = new MotorControllerGroup(rightFrontTalon, rightBackTalon);

    
    this.controller = m_controller;
    
    //Gyros
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.calibrate();

    //position estimator 
    poseEstimator = new DifferentialDrivePoseEstimator(Drive_Constants.trackWidth, new Rotation2d(gyro.getRoll()),Drive_Constants.initialLeftDistance, Drive_Constants.initialRightDistance, new Pose2d());

    leftFrontTalon.setNeutralMode(NeutralMode.Coast);
    leftBackTalon.setNeutralMode(NeutralMode.Coast);
    rightBackTalon.setNeutralMode(NeutralMode.Coast);
    rightFrontTalon.setNeutralMode(NeutralMode.Coast);

    //Sets the talon to return integrated encoder values
    leftFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);  
    rightFrontTalon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 

    leftFrontTalon.setSelectedSensorPosition(0); //zeros encoders when it connects to robot code 
    rightFrontTalon.setSelectedSensorPosition(0); 

    //DriveTrain
    jMoneyDrive = new DifferentialDrive(rightDrive, leftDrive);
    //jMoneyDrive.setMaxOutput(1);
    

    //PID
    distanceController = new PIDController(Drive_Constants.dKP,Drive_Constants.dKI, Drive_Constants.dKD);
    rotationController = new PIDController(Drive_Constants.rKP,Drive_Constants.rKI, Drive_Constants.rKD);
    balanceController = new PIDController(Drive_Constants.bKP, Drive_Constants.bKI, Drive_Constants.bKD);
    voltPID = new SimpleMotorFeedforward(Drive_Constants.dKS, Drive_Constants.dKV, Drive_Constants.dKA);

    //isFinished for balance command 
    balanceIsFinished = () -> {
      if(balanceController.calculate(gyro.getRoll(), 0) < 0.05 && Math.abs(gyro.getRoll()) < 2 && Math.abs(gyro.getRawGyroY()) < 0.03)
    { 
        return true;
    }
    else
    {
        return false;
    }
    }; 

    balanceEndCommand = (balanceIsFinished) -> {
      jMoneyDrive.curvatureDrive(0, 0, false);
    };

    

    tagDriveEndCommands = (tagDriveriveIsFinished) -> {
      jMoneyDrive.curvatureDrive(0, 0, false);
    };

    
  }

  @Override
  public void periodic() {
    //System.out.println(gyro.getRoll());
    System.out.println(" lb " + leftBackTalon.getSupplyCurrent() +" lf "+ leftFrontTalon.getSupplyCurrent() + " rb " + rightBackTalon.getSupplyCurrent() + " rf " + rightFrontTalon.getSupplyCurrent());
    
    //System.out.println(poseEstimator.getEstimatedPosition());

    // This method will be called once per scheduler run
    if (controller != null && RobotState.isTeleop()) {
      arcadeDrive(controller);
      timer.reset();
    }
    else{
      if(RobotState.isDisabled()) 
      {
        if(timer.get() <= 0.0)
        {
          setBrake(true);
          timer.reset();
          timer.start();
        }
        else if(timer.hasElapsed(5))
        {
          setBrake(false);
        }
      }
    }
    updatePoseEstimator();

    
}

  public void setBrake(Boolean brake) {
    if (brake) {
      leftFrontTalon.setNeutralMode(NeutralMode.Brake);
      leftBackTalon.setNeutralMode(NeutralMode.Brake);
      rightBackTalon.setNeutralMode(NeutralMode.Brake);
      rightFrontTalon.setNeutralMode(NeutralMode.Brake);
    }
    else
    {
      leftFrontTalon.setNeutralMode(NeutralMode.Coast);
      leftBackTalon.setNeutralMode(NeutralMode.Coast);
      rightBackTalon.setNeutralMode(NeutralMode.Coast);
      rightFrontTalon.setNeutralMode(NeutralMode.Coast); 
    }
    
  }

  //Returns wheel speeds of motors in meters per second
  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    //Speed = sensor count per 100 ms * distance per count * 10 (converts 100 ms to s) 
    double leftSpeed = leftFrontTalon.getSelectedSensorVelocity()*Drive_Constants.distancePerPulse_TalonFX*10*1.2 ; 
    double rightSpeed = rightFrontTalon.getSelectedSensorVelocity()*Drive_Constants.distancePerPulse_TalonFX*10;
   

    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }
  //Used by auto builder to run a path 
  public void setSpeeds(double leftSpeed, double rightSpeed)
  {
    leftSpeed*= .85;
    jMoneyDrive.curvatureDrive(leftSpeed, leftSpeed*.2, false);
    //jMoneyDrive.tankDrive(leftSpeed, rightSpeed);

    System.out.println("leftSpeed" + leftSpeed);
    System.out.println("rightSpeed" + rightSpeed);
    leftSpeed *= .75; //Accounts for crooked drivebase
    leftSpeed *= 2; //Max drivebase speed is likely around 5m/s, so when input 5m/s sets motor to 1.0(max)
    rightSpeed *= 2; 
  
  }
  //Sets the volts of each motor 
  /* 
  public void setVolts(double leftVolts, double rightVolts)
  {
    leftVolts *= .6;
    rightVolts *= .6;
    leftVolts -= 0.1*leftVolts;
    leftDrive.setVoltage(-leftVolts);
    rightDrive.setVoltage(-rightVolts);
   // System.out.println(getWheelSpeeds());
  }
  */

  //Used by the bot to drive -- calls upon adjust method to reduce error. Is used by the DefaultDrive command to drive in TeleOp
  public void arcadeDrive(Joystick controller) {
   
    //Gets controller values
    double speed = controller.getRawAxis(1);
    double rotate = -controller.getRawAxis(4);
    if (speed > 0.05 || rotate >0.05) {
      setBrake(false);
  }
  
  else {
    setBrake(true);      
  }
    speed = adjust(speed);
    rotate = adjust(rotate);
    speed = rateLimiter.calculate(speed);
    //rotate = rotateLimiter.calculate(rotate);
    double speedLeft;
    double speedRight;
    if (rotate > 0) {
      speedLeft = speed+Math.abs(rotate*0.5);
      speedRight = speed-Math.abs(rotate*0.5);
    } else if (rotate < 0) {
      speedLeft = speed-Math.abs(rotate*0.5);
      speedRight = speed+Math.abs(rotate*0.5);
    } else {
      speedLeft = speed;
      speedRight = speed;
    }

    /*
      if(Math.abs(rotate) < .1 && Math.abs(speed) <.5)
      {
      rotate += .1*speed; //before both were 0.1*speed
      }
      else if (Math.abs(rotate) < .1 && Math.abs(speed) >= 0.5){ //when driving straight 
       rotate += 0.1*speed;  //to fix the driveabse veering off to left  (soft fix for physcial problem)
      }*/

     
    //System.out.println(getWheelSpeeds());
    /*if (controller.getRawButton(XboxController.Button.kRightBumper.value)){ //Need to find the number
      jMoneyDrive.curvatureDrive(-speed/Drive_Constants.driveSensitivity, rotate/Drive_Constants.turnSensitivity , true);
    }
    else if(controller.getRawButton(XboxController.Button.kLeftBumper.value)){
      jMoneyDrive.curvatureDrive(speed/(Drive_Constants.driveSensitivity*5), rotate/(Drive_Constants.turnSensitivity) , true);
    }
    else{
    jMoneyDrive.curvatureDrive(speed/Drive_Constants.driveSensitivity, rotate/Drive_Constants.turnSensitivity  , true);
    }*/
    jMoneyDrive.tankDrive(speedLeft, speedRight);
  }

  //Also not required but stops drifiting and gurantees max speed
  public double adjust(double axis) {
    if (Math.abs(axis)<Drive_Constants.errormargin) {return 0.0;}
    if (Math.abs(axis)>(1-Drive_Constants.errormargin)) {return (Math.abs(axis)/axis);}
    return axis;
  }

  //Automatically Balances on charge station using gyro measurements
  public double balance()
  {
    double forwardSpeed = -balanceController.calculate(gyro.getRoll(), 0); //Calculates forward speed using PID
    jMoneyDrive.curvatureDrive(forwardSpeed,  forwardSpeed*0.05  , false);
    return forwardSpeed;
  }
  
  //Returns the current global pose estimate of robot
  public Pose2d getPose()
  {
    return poseEstimator.getEstimatedPosition(); 
  }

  //Resets global pose estimator based on a position parameter, gyro and encoder have no effect - used by auto
  public void resetPose(Pose2d pose)
  {
   // leftFrontTalon.setSelectedSensorPosition(0);
    //rightFrontTalon.setSelectedSensorPosition(0);
    gyro.reset();
    poseEstimator.resetPosition(new Rotation2d(0), 0, 0, pose);
  }

  public void drive(double speed, double rotate) {
    jMoneyDrive.curvatureDrive(speed, rotate, true);
  }

  public void updatePoseEstimator(){
    
    //Make sure timer delay is added if needed, could need because of motor delays from inversion
    double leftFrontEncoder = -leftFrontTalon.getSelectedSensorPosition() * Drive_Constants.distancePerPulse_TalonFX*.83;
    double rightFrontEncoder = rightFrontTalon.getSelectedSensorPosition() * Drive_Constants.distancePerPulse_TalonFX;
   // System.out.println("gyro" + gyro.getYaw());
    //System.out.println("left encoder" + leftFrontEncoder + " r " + rightFrontEncoder);
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d(), leftFrontEncoder, rightFrontEncoder);
     //ad gyro value
  } 
}



