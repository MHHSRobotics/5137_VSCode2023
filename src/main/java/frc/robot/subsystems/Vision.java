package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Vision_Constants;

public class Vision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private static PhotonPoseEstimator ar1CamPoseEstimator;
  private static PhotonPoseEstimator ar2CamPoseEstimator;

  private static PhotonCamera ar1Camera;
  private static PhotonCamera ar2Camera;

  public Vision() {
    //Initiallizes the camera being run with photovision, using the proper camera name
    ar1Camera = new PhotonCamera("AR1");
    ar2Camera = new PhotonCamera("AR2");

    //Pose estimators using each camera instance, make sure to update where the camera is on the robot in Constants.

    //Sets the April Tag field to the 2023 field. Uses try and catch to make sure field loading doesn't crash program.
    try {
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      System.out.println("Successfully loaded field layout.");
      System.out.println(aprilTagFieldLayout);
    } catch (IOException e) {
      e.printStackTrace();
      System.out.println("Failed to load field layout.");
    }

    ar1CamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar1Camera, Vision_Constants.robotToAR1Cam);
    ar2CamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, ar2Camera, Vision_Constants.robotToAR2Cam);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
  }

  public Optional<EstimatedRobotPose> getPoseFromCamera(PhotonPoseEstimator camera, Pose2d referencePose) {
    camera.setReferencePose(referencePose);
    return camera.update();
  }

  public Pose2d getNearestAlign(String targetName, Pose2d robotPose, boolean alliance) {
    int alignIndex = -1;
    double minDiff = Double.MAX_VALUE;
    double currentDiff;
    Pose2d[][] array = Vision_Constants.alignArray;

    int start;
    int end;
    int index;

    //Sets targets based on alliance
    if (alliance) { //True == red, False == blue
      start = 0;
      end = 3;
    } else {
      start = 5;
      end = 8;
    }

    //Sets targets based on position on the field
    switch (targetName) {
      case ("right"): {index = 0;}
      case ("middle"): {index = 1;}
      case ("left"): {index = 2;}
      default: {index = 1;}
    }

    //Gets the best pose based off of selection
    for (int i = start; i < end; i++) {
      currentDiff = Math.abs(array[i][index].getY() - robotPose.getY());
      if (currentDiff < minDiff) {
        minDiff = currentDiff;
        alignIndex = i;
      }
    }
    return array[alignIndex][0];
  }
}
