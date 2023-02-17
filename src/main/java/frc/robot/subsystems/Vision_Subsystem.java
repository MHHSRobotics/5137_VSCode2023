package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;


public class Vision_Subsystem extends SubsystemBase
{
  AprilTagFieldLayout aprilTagFieldLayout;
  PhotonPoseEstimator lifeCamPoseEstimator;
  
  public Vision_Subsystem()
  {
  
    //Initializes the camera being run with photonVision, using the proper camera name 
    PhotonCamera lifeCamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    lifeCamPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, lifeCamera, Constants.robotToLifeCam);

    
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




  public Optional<EstimatedRobotPose> getPoseFromLifeCamCamera(Pose2d referencePose) 
  {
    lifeCamPoseEstimator.setReferencePose(referencePose);
    return lifeCamPoseEstimator.update();
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
    











