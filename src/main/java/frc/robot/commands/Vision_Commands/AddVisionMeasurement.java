package frc.robot.commands.Vision_Commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;
import frc.robot.subsystems.Vision_Subsystem;

public class AddVisionMeasurement extends CommandBase {

  DriveBase_Subsystem driveBase_Subsystem;
  Vision_Subsystem vision_Subsystem;
  Pose2d estimatedPose;
  double timestamp;

  public AddVisionMeasurement(DriveBase_Subsystem driveBase_Subsystem, Vision_Subsystem vision_Subsystem) {
    this.driveBase_Subsystem = driveBase_Subsystem;
    this.vision_Subsystem = vision_Subsystem;
    addRequirements(vision_Subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Optional<EstimatedRobotPose> lifeCamResult = vision_Subsystem.getPoseFromLifeCamCamera(driveBase_Subsystem.getPose());

    if (lifeCamResult.isPresent()) {
      estimatedPose = lifeCamResult.get().estimatedPose.toPose2d();
      timestamp = lifeCamResult.get().timestampSeconds;
      driveBase_Subsystem.addVisionMeasurement(estimatedPose, timestamp);
    }
    }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}