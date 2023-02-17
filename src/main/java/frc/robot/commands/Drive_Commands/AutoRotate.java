package frc.robot.commands.Drive_Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase_Subsystem;

public class AutoRotate extends CommandBase{
    Pose2d targetPose;
    DriveBase_Subsystem driveBase_Subsystem;
    double motorSpeed;

    public AutoRotate(DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose)
    {
        this.targetPose = targetPose;
        this.driveBase_Subsystem = driveBase_Subsystem;
        addRequirements(driveBase_Subsystem);
    }


  @Override 
   public void initialize()
   {}

  @Override
  public void execute()
  {
    motorSpeed = driveBase_Subsystem.autoRotate(targetPose);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if(motorSpeed < 0.1)
    {
        return true;
    }
    else
    {
        return false;
    }
  }
}
