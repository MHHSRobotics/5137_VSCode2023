package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.LED_Subsystem;

public class LED_Commands {
    LED_Subsystem leds;

    public LED_Commands(LED_Subsystem k_leds) {
        this.leds = k_leds;
    }

    public Command coneLEDS() {
        return new InstantCommand(() -> {leds.setTeleOp("cone");});
    }

    public Command cubeLEDS() {
        return new InstantCommand(() -> {leds.setTeleOp("cube");;});
    }
}