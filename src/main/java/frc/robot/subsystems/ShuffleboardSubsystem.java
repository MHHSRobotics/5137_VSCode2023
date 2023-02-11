package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class ShuffleboardSubsystem extends SubsystemBase{

private AprilTagSubsystem aprilTagSubsystem;
private DriveBaseSubsystem driveBaseSubsystem;

public ShuffleboardSubsystem()
{
    aprilTagSubsystem = RobotContainer.aprilTagSubsystem;
    driveBaseSubsystem = RobotContainer.driveBase_Subsystem;
}

@Override
public void periodic()
{
    SmartDashboard.putBoolean("April Tag Detected",aprilTagSubsystem.photonCamera.getLatestResult().hasTargets());
    SmartDashboard.putNumber("April Tag ID",aprilTagSubsystem.photonCamera.getLatestResult().getBestTarget().getFiducialId());
    SmartDashboard.putString("Current Position",aprilTagSubsystem.robotPose.toString());
}




}