package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
 
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;
import frc.robot.constants.Controller_Constants.*;

public class Drive_Subsystem extends SubsystemBase {
  private static WPI_TalonFX leftFrontMotor;
  private static WPI_TalonFX leftBackMotor;
  private static WPI_TalonFX rightFrontMotor;
  private static WPI_TalonFX rightBackMotor;

  private static MotorControllerGroup leftDrive;
  private static MotorControllerGroup rightDrive;

  public static DifferentialDrive jMoneyDrive;
  public SimpleMotorFeedforward voltPID;

  private static Boolean arcadeDriveActive;

  //vision
  public static DifferentialDrivePoseEstimator poseEstimator;
  public static AHRS gyro;
  
  Pose2d estimatedPose;
  double timeStamp;

  private Joystick controller;

  public Drive_Subsystem(Joystick controller) {
    leftFrontMotor = new WPI_TalonFX(Drive_Constants.leftFrontPort);
    leftBackMotor = new WPI_TalonFX(Drive_Constants.leftBackPort);
    rightFrontMotor = new WPI_TalonFX(Drive_Constants.rightFrontPort);
    rightBackMotor = new WPI_TalonFX(Drive_Constants.rightBackPort);

    leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    jMoneyDrive = new DifferentialDrive(leftDrive, rightDrive);
    voltPID = new SimpleMotorFeedforward(Drive_Constants.dKS, Drive_Constants.dKV, Drive_Constants.dKA);

    arcadeDriveActive = false;

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.calibrate();
    poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(Units.inchesToMeters(Drive_Constants.trackWidth)), new Rotation2d(gyro.getRoll()), Drive_Constants.initialLeftDistance, Drive_Constants.initialLeftDistance, new Pose2d());

    this.controller = controller;
  }

  @Override
  public void periodic() {
    if (arcadeDriveActive) {
      arcadeDrive();
    }
  }

  private void arcadeDrive() {
    Double speed = adjust(controller.getRawAxis(PS4_Constants.LYPort));
    Double rotate = adjust(controller.getRawAxis(PS4_Constants.RXPort));
    jMoneyDrive.curvatureDrive(-speed/Drive_Constants.driveSensitivity, -rotate/Drive_Constants.turnSensitivity, true);
  }

  private Double adjust(Double x) {
    if (Math.abs(x) < 0.1) {return 0.0;}
    else if (Math.abs(x) > 0.9) {return Math.abs(x)/x;}
    else {return x;}
  }

  public void drive(Double speed, Double rotate) {
    arcadeDriveActive = false;
    jMoneyDrive.curvatureDrive(speed, rotate, true);
  }

  public void enableArcadeDrive() {
    arcadeDriveActive = true;
  }

  public Boolean getArcadeDriveEnabled() {
    return arcadeDriveActive;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftSpeed = leftFrontMotor.getSelectedSensorVelocity()*Drive_Constants.distancePerPulse_TalonFX*10;
    double rightSpeed = rightFrontMotor.getSelectedSensorVelocity()*Drive_Constants.distancePerPulse_TalonFX*10;
    return new DifferentialDriveWheelSpeeds(leftSpeed, rightSpeed);
  }

  public void setVolts(double leftVolts, double rightVolts) {
    leftVolts *= .2;
    rightVolts *= .2;
    leftVolts -= 0.1*leftVolts; //Accounting for wheel offset

    leftDrive.setVoltage(-leftVolts);
    rightDrive.setVoltage(-rightVolts);
  }

  //vision and pose 
  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void updatePoseEstimator(){
    double leftFrontEncoder = leftFrontMotor.getSelectedSensorPosition() * Drive_Constants.distancePerPulse_TalonFX;
    double rightFrontEncoder = rightFrontMotor.getSelectedSensorPosition() * Drive_Constants.distancePerPulse_TalonFX;

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), new Rotation2d((double)gyro.getRoll()), leftFrontEncoder, rightFrontEncoder);
  }

  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds){
    Optional<EstimatedRobotPose> ar1CamResult = Vision_Subsystem.getPoseFromCamera(Vision_Subsystem.ar1CamPoseEstimator, this.getPose()); 
    Optional<EstimatedRobotPose> ar2CamResult = Vision_Subsystem.getPoseFromCamera(Vision_Subsystem.ar2CamPoseEstimator, this.getPose()); 

    if (ar1CamResult.isPresent()){
      estimatedPose = ar1CamResult.get().estimatedPose.toPose2d();
      timeStamp = ar1CamResult.get().timestampSeconds;
      this.addVisionMeasurement(estimatedPose, timeStamp);
    }
    if (ar2CamResult.isPresent()){
      estimatedPose = ar2CamResult.get().estimatedPose.toPose2d();
      timeStamp = ar2CamResult.get().timestampSeconds;
      this.addVisionMeasurement(estimatedPose, timeStamp);
    }

  }

  public void resetPose(Pose2d pose){
    poseEstimator.resetPosition(new Rotation2d(gyro.getRoll()), 0, 0, pose);
  }
}