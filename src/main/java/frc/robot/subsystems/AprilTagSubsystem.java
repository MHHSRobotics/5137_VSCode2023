package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AprilTagSubsystem extends SubsystemBase{
    
//Creates the camera that will be used 
PhotonCamera photonCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

private double previousPipelineTimestamp = 0;

//Guys why wont this resolve
//AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));


//Where our Camera is on the robot 
Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 


public AprilTagSubsystem()
{
 
  //Sets LED/"Lime" to off 
photonCamera.setLED(VisionLEDMode.kOff);
photonCamera.setDriverMode(false);
photonCamera.setPipelineIndex(2);

}

@Override
  public void periodic() {
  

  

    //Creates a new PhotonPoseEstimator that can give us an estimate pose  
   // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_LAST_POSE, photonCamera, robotToCam);
// Rescans for the best target
    //Gathers latest result / "scan"
    var pipelineResult = photonCamera.getLatestResult();
    
    //Records the Time of the latest scan
    var resultTimestamp = pipelineResult.getTimestampSeconds();


  
   
    //Makes sure that this result has not already been processed and confirms that the scan has targets present
    if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      //Sets current scan to previous to prevent rescanning
        previousPipelineTimestamp = resultTimestamp;
        //Sets theh target used in AprilTag system to the best target in the "Scan"
      var target = pipelineResult.getBestTarget();
      //Records the id of the best target
      System.out.println("Yaw: " + target.getYaw());
      System.out.println("Pitch: " + target.getPitch());
      
      System.out.println("Ambiguity: " + target.getPoseAmbiguity());
      

      var fiducialId = target.getFiducialId();
      
      
      }
    }
    










 //Constructur for PhotonPoseEstimator
}
