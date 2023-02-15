package frc.robot.commands.Intake_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOnReverse extends CommandBase {
    public IntakeOnReverse() {
        addRequirements(RobotContainer.intake_Subsystem);
     }
 
     @Override
     public void execute() {
        if (!RobotContainer.intake_Subsystem.intakeActive) {
            RobotContainer.intake_Subsystem.extendIntake();
           }
        RobotContainer.intake_Subsystem.runIntake(false);
     }
 
     @Override
     public boolean isFinished() {
         return true;
     }
}
