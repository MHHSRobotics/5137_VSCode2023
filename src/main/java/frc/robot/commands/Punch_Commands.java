package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Punch_Subystem;
import edu.wpi.first.wpilibj.Timer;

public class Punch_Commands {
    Punch_Subystem punch;
    Timer time;

    public Punch_Commands(Punch_Subystem punch) {
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
                punch.punchLeft();
                try {
                    Thread.sleep(13, 500000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                punch.punchRight();
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
                    if (time.hasElapsed(1)) {
                        return true;
                    } else {
                        return false;
                    }
                }  
            },
            
            punch);
    }
}