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
import edu.wpi.first.math.util.Units;
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
public static Pose2d robotPose;

public boolean firstRotated = false;
public boolean secondRotated = false;
public boolean drivedTo = false;

public AprilTagSubsystem()
{
  //Sets tolerance of PID controllers so they dont have to be on 0.0000
//distanceController.setTolerance(.1);
rotationController.setTolerance(3);
//rotationController.enableContinuousInput(-360.0, 360.0);
//Pose estimator system for global pose estimation
poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, Constants.initialGyro, Constants.initialLeftDistance,Constants.initialRightDistance, Constants.initialPose);
//Used to call from driveBase subsystem non statically 
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


    //Test as an encoder??? Also make sure this uses the right motor/controller
 
    double wheelDiameter = Units.inchesToMeters(6); //Meters
    double distancePerPulseL = (wheelDiameter * Math.PI) / 4096.0;
    double leftDist = -DriveBaseSubsystem.leftTalon.getSelectedSensorPosition() * distancePerPulseL;
    
    
    double distancePerPulseR = (wheelDiameter * Math.PI) / 4096.0;
    double rightDist = DriveBaseSubsystem.rightTalon.getSelectedSensorPosition() * distancePerPulseR;

   
    //Gets the estimated global position of the robot for use later
    
    
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
        
        if(xboxc.getXButton())
        {
        System.out.println("Aligning to nearest left cone column");
        Pose2d align = getNearestAlign("left", robotPose);
        autoAlign(align);
      }
      if(xboxc.getXButtonReleased())
      {
        resetAlign();
      }
      if(xboxc.getYButton())
      {
        System.out.println("Aligning to nearest cube column");
        Pose2d align = getNearestAlign("middle", robotPose);
        System.out.println("alignPose" + align);
        autoAlign(align);
      }
      if(xboxc.getYButtonReleased())
      {
        resetAlign();
      }
      if(xboxc.getBButton())
      {
        System.out.println("Aligning to nearest right cone column");
        Pose2d align = getNearestAlign("right", robotPose);
        autoAlign(align);
      }
      if(xboxc.getBButtonReleased())
      {
        resetAlign();
      }
      
    //Prints when a detected tag moves out of frame/no longer detected
    }
    else if(resultTimestamp != previousPipelineTimestamp  && currentId != 0 && pipelineResult.hasTargets() == false)
    {
      currentId = 0;
      System.out.println("No more targets detected...");
      
    }
    updatePose(leftDist, rightDist);
    robotPose = poseEstimator.getEstimatedPosition();
    System.out.println(robotPose);
    //Updates the pose using vision measurements, gyro measurements, and encoders (when added)
    



  }

  //Gets the nearest right cone, cube, or left cone column. DO not call when halfway or farther across the field! 
  public Pose2d getNearestAlign(String targetName, Pose2d robotPose)
  {
    int alignIndex = -1;
    double minDiff = Double.MAX_VALUE;
    double currentDiff;
    Pose2d[][] array = Constants.alignArray;
    if(robotPose.getX() <= 8.27)
    {
      if(targetName.equals("right"))
      {
      for(int i = 5; i < 8; i++)
      {
      currentDiff = Math.abs(array[i][0].getY() - robotPose.getY());
       if(currentDiff < minDiff)
       {
        minDiff = currentDiff;
        alignIndex = i;
       }
      }
      return array[alignIndex][0];
      }
      if(targetName.equals("middle"))
      {
      for(int i = 5; i < 8; i++)
      {
      currentDiff = Math.abs(array[i][1].getY() - robotPose.getY());
       if(currentDiff < minDiff)
       {
        minDiff = currentDiff;
        alignIndex = i;
       }
      }
      return array[alignIndex][1];
      }
      if(targetName.equals("left"))
      {
      for(int i = 5; i < 8; i++)
      {
      currentDiff = Math.abs(array[i][2].getY() - robotPose.getY());
       if(currentDiff < minDiff)
       {
        minDiff = currentDiff;
        alignIndex = i;
       }
      }
      return array[alignIndex][2];
      }
      

    
  }
  else if(robotPose.getX() > 8.27)
      {
        if(targetName.equals("right"))
        {
        for(int i = 0; i < 3; i++)
        {
        currentDiff = Math.abs(array[i][0].getY() - robotPose.getY());
         if(currentDiff < minDiff)
         {
          minDiff = currentDiff;
          alignIndex = i;
         }
        }
        return array[alignIndex][0];
        }
        if(targetName.equals("middle"))
        {
        for(int i = 0; i < 3; i++)
        {
        currentDiff = Math.abs(array[i][1].getY() - robotPose.getY());
         if(currentDiff < minDiff)
         {
          minDiff = currentDiff;
          alignIndex = i;
         }
        }
        return array[alignIndex][1];
        }
        if(targetName.equals("left"))
        {
        for(int i = 0; i < 3; i++)
        {
        currentDiff = Math.abs(array[i][2].getY() - robotPose.getY());
         if(currentDiff < minDiff)
         {
          minDiff = currentDiff;
          alignIndex = i;
         }
        }
        return array[alignIndex][2];
      }
    }
    return array[3][0];
  }
  

  public void resetAlign()
  {
  firstRotated = false;
  secondRotated = false;
  drivedTo = false;
  }

  public void autoAlign(Pose2d targetPose)
  {
    if (firstRotated == false)
    {
      firstRotated = autoRotate(targetPose);
    }
    else if(drivedTo == false)
    {
      drivedTo = autoDriveForward(targetPose);

    }
    else if(secondRotated == false)
    {
      secondRotated = autoRotate(targetPose);
    }
  }

//Only supply the variables first and second rotated
  public boolean autoRotate(Pose2d targetPose)
  {
    if(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees() != 0 )
    {
      double rotationSpeed = -rotationController.calculate(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(), 0.0);
      System.out.println("Rotation speed: " + rotationSpeed);
      driveBaseSubsystem.testDrive.tankDrive(rotationSpeed,-rotationSpeed);
      if (PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees() == 0 || rotationSpeed == 0)
      {
        return true;
      }
      return false;
    }
    else
    {
      return true;
    }
    
  }


  public boolean autoDriveForward(Pose2d targetPose)
  {
    if(PhotonUtils.getDistanceToPose(robotPose, targetPose) != 0 )
    {
      double forwardSpeed = distanceController.calculate(PhotonUtils.getDistanceToPose(robotPose, targetPose), 0.0);
      System.out.println("Forward speed: " + forwardSpeed);
      driveBaseSubsystem.driveStraight(forwardSpeed);
      if (PhotonUtils.getDistanceToPose(robotPose, targetPose) == 0 || forwardSpeed == 0)
      {
        return true;
      }
      return false;
    }
    else
    {
      return true;
    }
  }
  

  public void updatePose(double leftDist, double rightDist) 
  {
 
    poseEstimator.update(DriveBaseSubsystem.gyro.getRotation2d(), leftDist, rightDist);
    
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
    










