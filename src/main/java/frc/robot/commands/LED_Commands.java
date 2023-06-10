package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.LED_Subsystem;

public class LED_Commands {
    LED_Subsystem leds;

    public LED_Commands(LED_Subsystem k_leds) {
        this.leds = k_leds;
    }

    public Command coneLEDS() {
        return new InstantCommand(() -> {
            leds.signal("Cone");
        });
    }

    public Command cubeLEDS() {
        return new InstantCommand(() -> {
            leds.signal("Cube");
        });
    }

    public Command emote() {
        return new InstantCommand(() -> {
            leds.signal("Green");
        });
    }

    public Command hotPink() {
        return new InstantCommand(() -> {
            leds.signal("Pink");
        });
    }
}