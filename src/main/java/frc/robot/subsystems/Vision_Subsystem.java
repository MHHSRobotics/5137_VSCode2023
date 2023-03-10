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

public class Vision_Subsystem extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  public static PhotonPoseEstimator ar1CamPoseEstimator;
  public static PhotonPoseEstimator lifeCamPoseEstimator;

  public static PhotonCamera ar1Camera;
  public static PhotonCamera lifeCamera;

  public Vision_Subsystem() {
    //Initiallizes the camera being run with photovision, using the proper camera name
    ar1Camera = new PhotonCamera("AR1");
    lifeCamera = new PhotonCamera("LifeCam");

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
    lifeCamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, lifeCamera, Vision_Constants.robotToAR2Cam);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
  }

  public static Optional<EstimatedRobotPose> getPoseFromCamera(PhotonPoseEstimator camera, Pose2d referencePose) {
    camera.setReferencePose(referencePose);
    return camera.update();
  }

  public Pose2d getNearestAlign(String targetName, Pose2d robotPose) {
    int alignIndex = -1;
    double minDiff = Double.MAX_VALUE;
    double currentDiff;
    Pose2d[][] array = Vision_Constants.alignArray;

    int leftmostTag;
    int rightmostTag;
    int column;

    //Sets targets based on alliance
    if (robotPose.getX() <= 8.27 ) { //blue
      leftmostTag = 5;
      rightmostTag = 8;
    } else { //red
      leftmostTag = 0;
      rightmostTag = 3;
    }

    //Sets targets based on position on the field
    switch (targetName) {
      case ("right"): {column = 2;}
      case ("middle"): {column = 1;}
      case ("left"): {column = 0;}
      default: {column = 1;}
    }

    //Gets the best pose based off of selection
    for (int i = leftmostTag; i < rightmostTag; i++) {
      currentDiff = Math.abs(array[i][column].getY() - robotPose.getY());
      if (currentDiff < minDiff) {
        minDiff = currentDiff;
        alignIndex = i;
      }
    }
    return array[alignIndex][column];
  }
}
