package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subystem;

public class IntakeExtend extends CommandBase {
    double timestamp;
     Intake_Subystem intake_Subystem;

    public IntakeExtend(Intake_Subystem intake_Subystem) {

        this.intake_Subystem = intake_Subystem;
        addRequirements(intake_Subystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake on unless intake is currently off  
         intake_Subystem.extendIntake();
     }
 
     @Override
     public boolean isFinished() {
        //Returns finished if the time is 0.5 seconds after command called
        return true;
     }
}