package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.RobotContainer;
import frc.robot.constants.Intake_Constants;
import frc.robot.simulation.SparkMaxWrapper;

public class Intake_Subystem extends SubsystemBase {
  private static SparkMaxWrapper intakeMotor;

  public Intake_Subystem() {
    intakeMotor = new SparkMaxWrapper(Intake_Constants.Port, MotorType.kBrushless);
  }

  //Wheels 
  public void runForward() {
    intakeMotor.set(Intake_Constants.Speed);
  }
 
  public void runReverse() {
    intakeMotor.set(-Intake_Constants.Speed);
  }

  public void stop() {
    intakeMotor.set(0.0);
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
