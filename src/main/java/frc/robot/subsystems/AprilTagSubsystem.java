package frc.robot.subsystems;

import java.io.IOException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import com.ctre.phoenixpro.signals.System_StateValue;

import java.util.Optional;



import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AprilTagSubsystem extends SubsystemBase{
    
//Creates the camera that will be used 
PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

private double previousPipelineTimestamp = 0;
private int currentId = 0;
private AprilTagFieldLayout aprilTagFieldLayout;
private PhotonPoseEstimator photonPoseEstimator;



//Where our Camera is on the robot 
Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 


public AprilTagSubsystem()
{
 
  //Sets LED/"Lime" to off 
photonCamera.setLED(VisionLEDMode.kOff);
photonCamera.setDriverMode(false);
//photonCamera.setPipelineIndex(0);




  
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
      
      
      EstimatedRobotPose estimatedRobotPose= photonPoseEstimator.update().get();
      Pose3d pose = estimatedRobotPose.estimatedPose;
      System.out.println(pose);
      



//Transform3d bestCameraToTarget = target.getBestCameraToTarget();
//Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
       //System.out.println(bestCameraToTarget);
    
      
      

    
      
     
    
      //Pose3d aprilToField = aprilTagFieldLayout.getTagPose(fiducialId).get();
      //System.out.println(aprilToField);


      // Calculate robot's field relative pose
     //Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),aprilToField , robotToCam);
   // System.out.println(robotPose);
      
   
   
   
   
      
     
      
      
      
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
    
    }
    










}
