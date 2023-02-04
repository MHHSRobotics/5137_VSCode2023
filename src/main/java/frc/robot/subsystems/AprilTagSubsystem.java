package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;



import java.util.Optional;



import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
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
private PhotonPoseEstimator photonPoseEstimator;
public XboxController xboxc = new XboxController(0);
//Calculates forward motor speed using distance to target
PIDController distanceController = new PIDController(Constants.dKP,Constants.dKI, Constants.dKD); //pid controller
//Calculates rotation speed using yaw 
PIDController rotationController = new PIDController(Constants.rKP,Constants.rKI, Constants.rKD);


//Where our Camera is on the robot 
Transform3d robotToCam = new Transform3d(new Translation3d(0.22, 0.0, 0.0), new Rotation3d(0,0,0)); 
DifferentialDrivePoseEstimator poseEstimator;
DriveBaseSubsystem driveBaseSubsystem;

public AprilTagSubsystem()
{

 
  poseEstimator = new DifferentialDrivePoseEstimator(Constants.trackWidth, Constants.initialGyro, Constants.initialLeftDistance,Constants.initialRightDistance, Constants.initialPose);
  driveBaseSubsystem = RobotContainer.driveBase_Subsystem;
  //Sets LED/"Lime" to off 
photonCamera.setLED(VisionLEDMode.kOff);
photonCamera.setDriverMode(false);
//photonCamera.setPipelineIndex(0); the number of the pipeline is on photon vision itself so choose from there I think

  
  try {
    aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    System.out.println ("Set the field");
    System.out.println(aprilTagFieldLayout);
  } catch (IOException e) {
    // TODO Auto-generated catch block
    e.printStackTrace();
    System.out.println ("DIdntworklol");
  }
  
  photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, photonCamera, robotToCam);

}

@Override
  public void periodic() {
  
  // Rescans for the best target
    //Gathers latest result / "scan"
    var pipelineResult = photonCamera.getLatestResult();
    
    //Records the Time of the latest scan
    var resultTimestamp = pipelineResult.getTimestampSeconds();


    //Makes sure that this result has not already been processed and confirms that the scan has targets present
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      //Sets current scan to previous to prevent rescanning
        previousPipelineTimestamp = resultTimestamp;
        //Sets the target used in AprilTag system to the best target in the "Scan"
      var target = pipelineResult.getBestTarget();
      //Records the id of the best target
      var fiducialId = target.getFiducialId();
      
      //Pose3d of robot
      EstimatedRobotPose estimatedRobotPose= photonPoseEstimator.update().get();
      Pose3d pose = estimatedRobotPose.estimatedPose;

    //Moves towards target / controls forward speed
    if (xboxc.getAButton()) {
          // Vision-alignment mode
          // Query the latest result from PhotonVision

          if (pipelineResult.hasTargets()) {//might not need this line actually
              // First calculate range
              
              double range = PhotonUtils.calculateDistanceToTargetMeters(
                                Constants.CAMERA_HEIGHT_METERS,
                                Constants.TARGET_HEIGHT_METERS,
                                Constants.CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(pipelineResult.getBestTarget().getPitch()));
                        
                        
                      
                // Use this range as the measurement we give to the PID controller.
                // -1.0 required to ensure positive PID controller effort _increases_ range
                double forwardSpeed = -distanceController.calculate(range, Constants.GOAL_RANGE_METERS);
              //System.out.println(forwardSpeed);
          
            }
   
          }
          //Controls rotation speed based upon yaw
          if (xboxc.getXButton()) {
           
  
            if (pipelineResult.hasTargets()) {
              double rotationSpeed = -rotationController.calculate(target.getYaw(), 0);
              //System.out.println(rotationSpeed);
            }
    
          }
      
      
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
    
    updatePose(0, 0);
    //System.out.println(poseEstimator.getEstimatedPosition());


  }


  


  public void updatePose(double leftDist, double rightDist) {
 
    poseEstimator.update(driveBaseSubsystem.gyro.getRotation2d(), leftDist, rightDist);
    
    var result = photonCamera.getLatestResult();
    if (result.hasTargets()) {
      
        var target = result.getBestTarget();
       
        var tagId = target.getFiducialId();
        
        var tagPose = aprilTagFieldLayout.getTagPose(tagId).get();
        
        var imageCaptureTime = result.getTimestampSeconds();
       
        var camToTargetTrans = target.getBestCameraToTarget();
        System.out.println(camToTargetTrans);
        
        var camPose = tagPose.transformBy(camToTargetTrans.inverse());
        
        var robotPose = camPose.transformBy(robotToCam).toPose2d();

        poseEstimator.addVisionMeasurement(robotPose, imageCaptureTime);
    }
}

}
    











