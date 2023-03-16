package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;

import frc.robot.constants.Punch_Constants;

public class Punch_Subsystem extends SubsystemBase {
    public static PneumaticsControlModule PCM;
    public static Compressor comp;
    public static Solenoid leftSolenoid;
    public static Solenoid rightSolenoid;
    
    public Punch_Subsystem() {
        PCM = new PneumaticsControlModule();
        comp = PCM.makeCompressor();
        leftSolenoid = PCM.makeSolenoid(Punch_Constants.leftPort);
        rightSolenoid = PCM.makeSolenoid(Punch_Constants.rightPort);
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
}
