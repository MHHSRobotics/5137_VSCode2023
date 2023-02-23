package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Intake_Constants;
import frc.robot.simulation.SparkMaxWrapper;

public class Intake extends SubsystemBase {

  private boolean intakeActive;
  private boolean intakeExtended;
  public static SparkMaxWrapper intakeMotor;

  public Intake() {
    intakeMotor = new SparkMaxWrapper(Intake_Constants.Port, MotorType.kBrushless);
  }

  public void runForward() {
    intakeMotor.set(Intake_Constants.Speed);
    intakeActive = true;
  }

  public void runReverse() {
    intakeMotor.set(-Intake_Constants.Speed);
    intakeActive = true;
  }

  public void stop() {
    intakeMotor.set(0.0);
    intakeActive = false;
  }
}
