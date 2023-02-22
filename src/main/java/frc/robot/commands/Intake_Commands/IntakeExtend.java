package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeExtend extends CommandBase {
    public IntakeExtend() {
        addRequirements(RobotContainer.intake_Subystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake on unless intake is currently off  
        RobotContainer.intake_Subystem.extendIntake();
     }
 
     @Override
     public boolean isFinished() {
        //Returns finished if the time is 0.5 seconds after command called
         return true;
     }
}