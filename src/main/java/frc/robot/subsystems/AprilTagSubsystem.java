package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase{
    
PhotonCamera photonCamera = new PhotonCamera("grizzlycam");
   
public AprilTagSubsystem()
{
photonCamera.setLED(VisionLEDMode.kOff);
}
//Will be used in periodic to prevent inefficient scanning
private double previousPipelineTimestamp = 0;

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
      var fiducialId = target.getFiducialId();i
    }
}
}
