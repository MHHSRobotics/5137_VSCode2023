package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOff extends CommandBase {
    public IntakeOff() {
        addRequirements(RobotContainer.intake_Subsystem);
     }
 
     @Override
     public void execute() {
        //doesn't toggle intake off unless intake is currently on
        //technically unnecessary but we're keeping it 
        if (RobotContainer.intake_Subsystem.intakeActive) {
            RobotContainer.intake_Subsystem.retractIntake();
        }
        RobotContainer.intake_Subsystem.stopIntake();
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
