package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clamp_Subsystem extends SubsystemBase {

    public Clamp_Subsystem() {
        
    }

    public void Clamp(boolean type) {
        if (type) {
            Pneumatics_Subsystem.clampSolenoid.set(true);

        } else {
            //Need to make cube not die
            Pneumatics_Subsystem.clampSolenoid.set(true);
        }
    }

    public void Release() {
        Pneumatics_Subsystem.clampSolenoid.set(false);
    }
}
