package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOff extends CommandBase {
    public IntakeOff() {
        addRequirements(RobotContainer.intake_Subystem);
     }
 
     @Override
     public void execute() {
        RobotContainer.intake_Subystem.stopIntake(); // Turns off intake
        RobotContainer.intake_Subystem.retractIntake(); //Pulls intake in
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
} 
