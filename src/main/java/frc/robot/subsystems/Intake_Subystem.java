package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.constants.Intake_Constants;
import frc.robot.objects.SparkMaxWrapper;

public class Intake_Subystem extends SubsystemBase {
  private static SparkMaxWrapper intakeMotor; 
  public static boolean intakeOveride; //this will override the arm automatically extending/rectracting the intake 

  public Intake_Subystem() {
    intakeOveride = false;
    intakeMotor = new SparkMaxWrapper(Intake_Constants.Port, MotorType.kBrushless);
  }

  //Wheels 
  public void runForward() {
    intakeMotor.set(Intake_Constants.Speed);
    intakeOveride = true;
  }
 
  public void runReverse() {
    intakeMotor.set(-Intake_Constants.Speed);
    intakeOveride = true;
  }

  public void runAutoReverse() {
    intakeMotor.set(-Intake_Constants.Speed*0.5);
    intakeOveride = true;
  }

  public void stop() {
    intakeMotor.set(0.0);
    intakeOveride = false;
  }

  //Pneumatics 
  public void extendIntake() {
    RobotContainer.pneumatics_Subsystem.enableIntake();
  }

  public void retractIntake() {
    RobotContainer.pneumatics_Subsystem.disableIntake();
  }

  public double getIntakeSpeed() {
    return intakeMotor.get();
  }
}
