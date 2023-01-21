package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase{
    
//Creates the camera that will be used 
PhotonCamera photonCamera = new PhotonCamera("grizzlycam");
private final AprilTagFieldLayout aprilTagFieldLayout;
//Will be used in periodic to prevent inefficient scanning
private double previousPipelineTimestamp = 0;


public AprilTagSubsystem()
{
  //Sets LED/"Lime" to off 
photonCamera.setLED(VisionLEDMode.kOff);

try {
  layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
  var alliance = DriverStation.getAlliance();
  layout.setOrigin(alliance == Alliance.Blue ?
      OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
} catch(IOException e) {
  DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
  layout = null;
}

this.aprilTagFieldLayout = layout;
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
        //Sets theh target used in AprilTag system to the best target in the "Scan"
      var target = pipelineResult.getBestTarget();
      //Records the id of the best target
      var fiducialId = target.getFiducialId();
      //get tag pose from layout, consider if the layout will be null if it failed to load
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
      //positiion of April Tag, if it is null
      if(target.getPoseAmbiguity() <= .2 && fiducialId != null && tagPose.isPresent()){
        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();//gets position
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT); //how high up/far back the camera is on the robot
      }
    }
}


//Change for where our camera is on OUR robot: Cam mounted facing forward, half a meter forward of center, half a meter up from center.
Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 

/* 
public poseEstimator(PhotonCamera photonCamera){
  this.photonCamera = photonCamera;
  //sets the camera to the camera we're using
  AprilTagFieldLayout layout;
  //gets the field layout from saved pdf
    try{
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargerUp.m_resourceFile);
      var alliance = DriverStation.getAlliance();
      //idk if we need this tbh because the team that made this had swerve drive so idk if our team needs this
      //but they did also use gyro to get position which we will have so idk ill just leave it for now and see
      //if we need it later :)
    }
}

*/





// Constructur for PhotonPoseEstimator
PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.CLOSEST_TO_LAST_POSE, photonCamera, robotToCam);
}
