package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;

import frc.robot.constants.Punch_Constants;

public class Punch_Subsystem extends SubsystemBase {
    private static PneumaticHub pH;
    private static Compressor comp;
    private static Solenoid leftSolenoid;
    private static Solenoid rightSolenoid;
    
    public Punch_Subsystem() {
        pH = new PneumaticHub(6);
        comp = pH.makeCompressor();
        leftSolenoid = pH.makeSolenoid(Punch_Constants.leftPort);
        rightSolenoid = pH.makeSolenoid(Punch_Constants.rightPort);
    }

    public void enableCompressor() {
        comp.enableAnalog(Punch_Constants.minPressure, Punch_Constants.maxPressure);
    }

    public void disableCompressor() {
        comp.disable();
    }

    public void punch() {
        leftSolenoid.set(true);
        rightSolenoid.set(true);
    }

    public void retract() {
        leftSolenoid.set(false);
        rightSolenoid.set(false);
    }

    public boolean getCompActive() {
        if (comp.isEnabled()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean getSolenoidsActive() {
        if (leftSolenoid.get() && rightSolenoid.get()) {
            return true;
        } else {
            return false;
        }
    }
}