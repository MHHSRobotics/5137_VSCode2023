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

  public Command balance(){
      return new FunctionalCommand(
        () -> {}, 
        () -> drive.balance(), 
        drive.balanceEndCommand, 
        drive.balanceIsFinished, 
        drive);
  }

  public Command setBrake(Boolean brake) {
    return new InstantCommand(() -> {drive.setBrake(brake);});
  }

  public Command driveForward() {
    return new InstantCommand(() -> {drive.drive(0.2, 0.0);});
  }

  public Command stop() {
    return new InstantCommand(() -> {drive.drive(0, 0);});
  }

}