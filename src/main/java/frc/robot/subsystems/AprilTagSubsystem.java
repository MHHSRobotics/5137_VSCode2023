package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;


public class AprilTagSubsystem extends SubsystemBase
{
    
  //public instance variables, may be used elswhere in robot
  public PhotonCamera photonCamera;
  public AprilTagFieldLayout aprilTagFieldLayout;
  public XboxController xboxc = new XboxController(0); //xbox controller to be called with methods
  public PIDController distanceController;
  public PIDController rotationController;
  public Pose2d robotPose;

  //private instance variables, will only be used locally
  private double previousPipelineTimestamp = 0;
  private int currentId = 0; //Holds the current april tag ID (when detected)
  private boolean firstRotated = false; //Determines if the first rotation is finished when autoAlign() is running
  private boolean secondRotated = false; //Determines if the second rotation is finished when autoAlign() is running
  private boolean drivedTo = false; //Determines if the driving is finished when autoAlign() is running

  //Will be used to reference the DriveBaseSubsystem
  private DriveBaseSubsystem driveBaseSubsystem = RobotContainer.driveBase_Subsystem;
  
  //Creates a new global position estimator, intializes it with track width of robot, initial sensor values, and an initial position. 
  private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, Constants.initialGyro, Constants.initialLeftDistance,Constants.initialRightDistance, Constants.initialPose);

  
  public AprilTagSubsystem()
  {
    //PID Controller for moving forward and backwards, with specified gains
    distanceController = new PIDController(Constants.dKP,Constants.dKI, Constants.dKD);
    distanceController.setTolerance(.1); //Sets PID tolerance range

    //PID Controller for rotating, with specified gains 
    rotationController = new PIDController(Constants.rKP,Constants.rKI, Constants.rKD);
    rotationController.setTolerance(3); //Sets PID tolerance range

    //Initializes the camera being run with photonVision, using the proper camera name 
    photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    photonCamera.setLED(VisionLEDMode.kOff); //Sets LED/"Lime" to off 
    photonCamera.setDriverMode(false); //Turns off driverMode in Photonvision
    
    //Sets the April Tag field to the 2023 field. Uses try and catch to make sure field loading doesn't crash program. 
    try 
    {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      System.out.println ("Set the field");
      System.out.println(aprilTagFieldLayout);
    } 
    catch (IOException e) 
    {
      e.printStackTrace();
      System.out.println ("The field Load Didn't Work");
    }
  }


@Override
  public void periodic() {

    var pipelineResult = photonCamera.getLatestResult(); //Gathers latest result from Camera
    var resultTimestamp = pipelineResult.getTimestampSeconds(); //Records the Time of the latest result
    
    //Makes sure that the latest result has not already been checked and that vision targets are present
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) 
    {
      previousPipelineTimestamp = resultTimestamp; //Sets result time to "previous" time to prevent rechecking
      var target = pipelineResult.getBestTarget(); //Gets the best vision target to be used from result
      var fiducialId = target.getFiducialId(); //Records the id of the best target
      
      //Prints information when a new tag is found, including Tag ID
      if(fiducialId != currentId)
      {
        currentId = fiducialId;
        System.out.println("New Best Target Detected!");
        System.out.println("Tag ID: " + currentId);
      }
    }
    //Prints when a tag moves out of frame - when targets are no longer detected
    else if(resultTimestamp != previousPipelineTimestamp  && currentId != 0 && pipelineResult.hasTargets() == false)
    {
      currentId = 0; //Sets current ID to 0 - null 
      System.out.println("No more targets detected...");
    }

    updatePose(DriveBaseSubsystem.leftDist, DriveBaseSubsystem.rightDist); //Updates the Pose estimator - passes encoder distances as paramaters
    robotPose = poseEstimator.getEstimatedPosition(); //Sets Pose2d robotPose to updated position from poseEstimator
    
    //TESTING ONLY: Runs autoAlign towards leftCone, cube, or rightCone depending on button input
    if(xboxc.getXButton())
    {
      Pose2d align = getNearestAlign("left", robotPose); //Gets nearest left cone alignPose
      autoAlign(align);
    }
    //Used to reset the autoAlign function if button is no longer held - gets ready for next use
    if(xboxc.getXButtonReleased())
    {
      resetAlign(); 
    }

    if(xboxc.getYButton())
    {
      Pose2d align = getNearestAlign("middle", robotPose); //Gets nearest cube alignPose
      autoAlign(align);
    }
    if(xboxc.getYButtonReleased())
    {
      resetAlign();
    }

    if(xboxc.getBButton())
    {
      Pose2d align = getNearestAlign("right", robotPose); //Gets nearest right cone alignPose
      autoAlign(align);
    }
    if(xboxc.getBButtonReleased())
    {
      resetAlign();
    }
  }

  
  //Automatically algins to an alignPose to score using a rotate/drive/rotate and booleans for sequencing
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


  //Reset align booleans used in autoAlign
  public void resetAlign()
  {
    firstRotated = false;
    secondRotated = false;
    drivedTo = false;
  }


  //Sets the rotate speed of the robot using a PID and the robotPose angle, returns a boolean wether it has rotated completely or not
  public boolean autoRotate(Pose2d targetPose)
  {
    if(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees() != 0 )
    {
      double rotationSpeed = -rotationController.calculate(PhotonUtils.getYawToPose(robotPose, targetPose).getDegrees(), 0.0); //Calculates rotation speed using PID
      System.out.println("Rotation speed: " + rotationSpeed); //Prints the rotation speed
      driveBaseSubsystem.testDrive.tankDrive(rotationSpeed,-rotationSpeed); //Sets the drivetrain to rotate using PID speed
      
      //If the rotation is completete either by PID speed being 0 or Yaw
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


  //Sets the forward speed of the robot using a PID and the robotPose distance, returns a boolean wether it has reached target or not
  public boolean autoDriveForward(Pose2d targetPose)
  {
    if(PhotonUtils.getDistanceToPose(robotPose, targetPose) != 0 )
    {
      double forwardSpeed = distanceController.calculate(PhotonUtils.getDistanceToPose(robotPose, targetPose), 0.0); //Calculates forward speed using PID
      System.out.println("Forward speed: " + forwardSpeed); //Prints the forward speed
      driveBaseSubsystem.driveStraight(forwardSpeed); //Sets the drivetraub to drive forward/backwards using PID speed
  
      //If the drive is completete either by PID speed being 0 or distance
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
  

  //Updates the global pose estimator with vision, encoder, and gyro values
  public void updatePose(double leftDist, double rightDist) 
  {
 
    poseEstimator.update(DriveBaseSubsystem.gyro.getRotation2d(), leftDist, rightDist); //gives sensor inputs to global pose estimator
    
    //If there is a vision target, update the global pose estimator with that
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) 
    {
      
      var target = result.getBestTarget();
       
      var tagId = target.getFiducialId();
        
      var tagPose = aprilTagFieldLayout.getTagPose(tagId).get();
        
      var imageCaptureTime = result.getTimestampSeconds();
       
      var camToTargetTrans = target.getBestCameraToTarget();
        
      var camPose = tagPose.transformBy(camToTargetTrans.inverse());
        
      var robotPose = camPose.transformBy(Constants.robotToCam).toPose2d();

      poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }
  }
  
  //Gets the nearest right cone, cube, or left cone column. DO not call when halfway or farther across the field!
  public Pose2d getNearestAlign(String targetName, Pose2d robotPose)
  {
    int alignIndex = -1;
    double minDiff = Double.MAX_VALUE;
    double currentDiff;
    Pose2d[][] array = Constants.alignArray;
    //If on the left half of the field/blue alliance
    if(robotPose.getX() <= 8.27)
    {
      //Checks which right cone alignPose is closest on the y axis
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
      //Checks which cube alignPose is closest on the y axis
      else if(targetName.equals("middle"))
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
      //Checks which left cone alignPose is closest on the y axis
      else if(targetName.equals("left"))
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
    //If on the right half of field / red alliance
    else if(robotPose.getX() > 8.27)
      {
        //Checks which right cone alignPose is closest on the y axis
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
        //Checks which cube alignPose is closest on the y axis
        else if(targetName.equals("middle"))
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
        //Checks which left cone alignPose is closest on the y axis
        else if(targetName.equals("left"))
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
}
    











