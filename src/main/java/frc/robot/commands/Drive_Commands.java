package frc.robot.commands;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Drive_Subsystem;



//import edu.wpi.first.math.geometry.Pose2d;

//import edu.wpi.first.wpilibj2.command.*;

//import frc.robot.subsystems.Drive_Subsystem;

 

 
public class Drive_Commands {
  private final Drive_Subsystem drive;

  public Drive_Commands (Drive_Subsystem drive) {
    this.drive = drive; 
  } 


  /*public class autoDrive(Pose2d targetPose){ //DriveBase_Subsystem driveBase_Subsystem, Pose2d targetPose
    return new FunctionalCommand(
      () -> {}, 
      () -> drive.autoDrive(targetPose), 
      () -> , 
      () -> true, 
      drive);
  }*/

  public Command tagDrive(Pose2d targetPose){
    return new FunctionalCommand(
      () -> {}, 
      () -> drive.tagDrive(targetPose), 
      drive.tagDriveEndCommands, 
      drive.tagDriveriveIsFinished(targetPose), 
      drive);
  }

  public Command balance(){
    /*return new FunctionalCommand(
      () -> {},
      () -> {drive.balance;},
      drive.balanceEndCommand,
      drive.balanceIsFinished,
      drive);*/
      return new FunctionalCommand(
        () -> {}, 
        () -> drive.balance(), 
        drive.balanceEndCommand, 
        drive.balanceIsFinished, 
        drive);
  }

}