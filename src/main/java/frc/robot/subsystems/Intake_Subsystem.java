package frc.robot.subsystems;

//import frc.robot.simulation.SparkMaxWrapper;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class Intake_Subsystem extends SubsystemBase {
    public boolean intakeActive = false;

    public static CANSparkMax intakeMotor;
    //public static SparkMaxWrapper intakeMotor;

    public Intake_Subsystem() {
        intakeMotor = new CANSparkMax(Constants.intakePort, MotorType.kBrushless);
        //intakeMotor = new SparkMaxWrapper(Constants.intakePort, MotorType.kBrushless);
    }

    public void runIntake(boolean direction) {
        if (direction) {
            intakeMotor.set(Constants.intakeSpeed);
        } else {
            intakeMotor.set(-Constants.intakeSpeed);
        }
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void extendIntake() {
        Pneumatics_Subsystem.intakeSolenoid.set(true);
        intakeActive = true;
    }

    public void retractIntake() {
        Pneumatics_Subsystem.intakeSolenoid.set(false);
        intakeActive = false;
    }
}
