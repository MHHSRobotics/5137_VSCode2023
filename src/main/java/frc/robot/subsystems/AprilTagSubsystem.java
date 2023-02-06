package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import org.photonvision.common.hardware.VisionLEDMode;



import java.util.Optional;



import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.math.controller;

public class AprilTagSubsystem extends SubsystemBase{
    
//Creates the camera that will be used 
PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

private double previousPipelineTimestamp = 0;
private int currentId = 0;
public AprilTagFieldLayout aprilTagFieldLayout;

public XboxController xboxc = new XboxController(0);
//Calculates forward motor speed using distance to target
PIDController distanceController = new PIDController(Constants.dKP,Constants.dKI, Constants.dKD); //pid controller
//Calculates rotation speed using yaw 
PIDController rotationController = new PIDController(Constants.rKP,Constants.rKI, Constants.rKD);


//Where our Camera is on the robot 
Transform3d robotToCam = new Transform3d(new Translation3d(0.22, 0.0, 0.0), new Rotation3d(0,0,0)); 
DifferentialDrivePoseEstimator poseEstimator;
DriveBaseSubsystem driveBaseSubsystem;
Pose2d robotPose;

public AprilTagSubsystem()
{

  poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, Constants.initialGyro, Constants.initialLeftDistance,Constants.initialRightDistance, Constants.initialPose);
  driveBaseSubsystem = RobotContainer.driveBase_Subsystem;
  //Sets LED/"Lime" to off 
photonCamera.setLED(VisionLEDMode.kOff);
photonCamera.setDriverMode(false);
//photonCamera.setPipelineIndex(0); the number of the pipeline is on photon vision itself so choose from there I think

  //Sets the April Tag field to the 2023 field
  try {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    System.out.println ("Set the field");
    System.out.println(aprilTagFieldLayout);
  } catch (IOException e) {
    
    e.printStackTrace();
    System.out.println ("Field Load Didn't Work");
  }
}


@Override
  public void periodic() {
    //Gets the estimated global position of the robot for use later
    robotPose = poseEstimator.getEstimatedPosition();
  
  // Rescans for the best target
    //Gathers latest result / "scan"
    var pipelineResult = photonCamera.getLatestResult();
    
    //Records the Time of the latest scan
    var resultTimestamp = pipelineResult.getTimestampSeconds();

    //Makes sure that this result has not already been processed and confirms that the scan has targets present
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) 
    {
      //Sets current scan to previous to prevent rescanning
        previousPipelineTimestamp = resultTimestamp;
        //Sets the target used in AprilTag system to the best target in the "Scan"
      var target = pipelineResult.getBestTarget();
      //Records the id of the best target
      var fiducialId = target.getFiducialId();
      
     
    //Print System for RioLog 
      //Prints message and tag id when a new tag is found
      if(fiducialId != currentId)
      {
        currentId = fiducialId;
        System.out.println("New Best Target Detected!");
        System.out.println("Tag ID: " + currentId);
      }
    //Prints when a detected tag moves out of frame/no longer detected
    }
    else if(resultTimestamp != previousPipelineTimestamp  && currentId != 0 && pipelineResult.hasTargets() == false)
    {
      currentId = 0;
      System.out.println("No more targets detected...");
        
    }
    
    //Updates the pose using vision measurements, gyro measurements, and encoders (when added)
    updatePose(0, 0);
  }


  public void autoMove(Pose2d targetPose)
  {
    autoRotate(targetPose);
    autoDriveForward(targetPose);
    autoRotate(targetPose);
    autoDriveForward(targetPose);
  }


  public void autoRotate(Pose2d targetPose)
  {
    while(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees() != 0 )
    {
    double rotationSpeed = -rotationController.calculate(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(), 0.0);
    
    driveBaseSubsystem.testDrive.tankDrive(-rotationSpeed,rotationSpeed);
    if(rotationSpeed == 0)
    {
    break;
    }
    }
  }


  public void autoDriveForward(Pose2d targetPose)
  {
    while(PhotonUtils.getDistanceToPose(robotPose, targetPose) != 0 )
    {
    double forwardSpeed = -distanceController.calculate(PhotonUtils.getDistanceToPose(robotPose, targetPose), 0.0);
    
    driveBaseSubsystem.driveStraight(forwardSpeed);
    if(forwardSpeed == 0)
    {
    break;
    }
    }
  }
  

  public void updatePose(double leftDist, double rightDist) 
  {
 
    poseEstimator.update(driveBaseSubsystem.gyro.getRotation2d(), leftDist, rightDist);
    
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) 
    {
      
      var target = result.getBestTarget();
       
      var tagId = target.getFiducialId();
        
      var tagPose = aprilTagFieldLayout.getTagPose(tagId).get();
        
      var imageCaptureTime = result.getTimestampSeconds();
       
      var camToTargetTrans = target.getBestCameraToTarget();
        
      var camPose = tagPose.transformBy(camToTargetTrans.inverse());
        
      var robotPose = camPose.transformBy(robotToCam).toPose2d();

      poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }
  }
}
    











