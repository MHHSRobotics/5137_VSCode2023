package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Punch_Subsystem;
import edu.wpi.first.wpilibj.Timer;

public class Punch_Commands {
    Punch_Subsystem punch;
    Timer time;

    public Punch_Commands(Punch_Subsystem punch) {
        this.punch = punch;
        time = new Timer();
    }

    public Command Punch() {
        return new FunctionalCommand (
            () -> {
                time.reset();
                time.start();
            },

            () -> {
                punch.punch();
            },

            new Consumer<Boolean>() {
                @Override
                public void accept(Boolean finished) {   
                    punch.retract();           
                }  
            },

            new BooleanSupplier() {
                @Override
                public boolean getAsBoolean() {
                    if (time.hasElapsed(2)) {
                        return true;
                    } else {
                        return false;
                    }
                }  
            },
            
            punch);
    }
}