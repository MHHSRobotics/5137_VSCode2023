package frc.robot.commands;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
//import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.Drive_Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;



//import edu.wpi.first.math.geometry.Pose2d;

//import edu.wpi.first.wpilibj2.command.*;

//import frc.robot.subsystems.Drive_Subsystem;

 

 
public class Drive_Commands {
  private final Drive_Subsystem drive;

  private Timer timer;

  public Drive_Commands (Drive_Subsystem drive) {
    this.drive = drive; 
    timer = new Timer();
    timer.reset();
  }

  public Command timedDrive(double seconds){
    return new FunctionalCommand(
      () -> {timer.reset(); timer.start();}, 
      () -> drive.setSpeeds(1, 1), 
      new Consumer<Boolean>(){
        @Override
        public void accept(Boolean finished) {   
            drive.setSpeeds(0, 0);          
        }  
      }, 
      new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            if (timer.hasElapsed(seconds)) {
                return true;
            } else {
                return false;
            }
        }  
    }, 
      drive);
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