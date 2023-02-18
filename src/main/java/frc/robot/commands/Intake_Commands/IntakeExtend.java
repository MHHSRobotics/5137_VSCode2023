package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeExtend extends CommandBase {
    double timestamp;
    public IntakeExtend() {
        addRequirements(RobotContainer.intake_Subsystem);
        
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake on unless intake is currently off  
         RobotContainer.intake_Subsystem.extendIntake();
         timestamp = Timer.getFPGATimestamp();
     }
 
     @Override
     public boolean isFinished() {
        //Returns finished if the time is 0.5 seconds after command called
         if(Timer.getFPGATimestamp() > timestamp + 0.5)
         {
            return true;
         }
         return false;
     }
}