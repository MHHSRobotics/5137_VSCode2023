package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.constants.LED_Constants;
import frc.robot.Robot;

public class LED_Subsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDSim ledSim;
    private static AddressableLEDBuffer ledBuffer;
    private static int pulse = 0;
    private static String type = "normal";

    //Middle led is 64

    public LED_Subsystem() {
        led = new AddressableLED(LED_Constants.Port);
        ledSim = new AddressableLEDSim(led);
        ledBuffer = new AddressableLEDBuffer(LED_Constants.Length);
        led.setLength(ledBuffer.getLength());
        startLED();
    }

    public void startLED() {
        led.start();
    }

    public void stopLED() {
        led.stop();
    }

    //Everything here needs to be fixed

    public void solidColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

    public void pulsingCG(double spacing, double speed) {
        for (var i = 0; i < LED_Constants.Length; i++) {
            if (Math.floor(i/spacing)%2 == 0) {
                double red = 200*((i%spacing)/spacing);
                double green = 0;
                double blue = 0;
                Color finalColor = new Color((int)red, (int)green, (int)blue);
                ledBuffer.setLED((i+(pulse/50))%150, finalColor);
            } else {
                double red = 100*((i%spacing)/spacing);
                double green = 40*((i%spacing)/spacing);
                double blue = 0;
                Color finalColor = new Color((int)red, (int)green, (int)blue);
                ledBuffer.setLED((i+(pulse/50))%150, finalColor);
            }
        }
        pulse += speed;
    }

    public void resetPulse() {
        pulse = 0;
    }

    public void solidCG(double speed) {
        if (pulse < 50) {
            solidColor(LED_Constants.Red);
        } else if (pulse < 100) {
            solidColor(LED_Constants.None);
        } else if (pulse < 150) {
            solidColor(LED_Constants.Gold);
        } else if (pulse < 200) {
            solidColor(LED_Constants.None);
        } else {
            pulse = 0;
        }
        pulse += speed;
    }

    public void pulsingColor(double speed, double spacing, Color color) {
        for (var i = 0; i < LED_Constants.Length; i++) {
                double red = 255*color.red*((i%spacing)/spacing);
                double green = 255*color.green*((i%spacing)/spacing);
                double blue = 255*color.blue*((i%spacing)/spacing);
                Color finalColor = new Color((int)red, (int)green, (int)blue);
                ledBuffer.setLED((i+(pulse/50))%150, finalColor);
        }
        pulse += speed;
    }

    public void flashingColor(double speed, Color color) {
        if (pulse < 50) {
            solidColor(color);
        } else if (pulse < 100) {
            solidColor(LED_Constants.None);
        } else {
            pulse = 0;
        }
        pulse += speed;
    }

    public void setTeleOp(String typ) {
        if (type == typ) {
            type = "normal";
        } else {
            type = typ;
        }
    }

    public void pulsingTele() {
        int speed;

        if (Robot.time.hasElapsed(105)) {
            speed = 50;
        } else if (Robot.time.hasElapsed(90)) {
            speed = 40;
        } else if (Robot.time.hasElapsed(60)) {
            speed = 30;
        } else if (Robot.time.hasElapsed(30)) {
            speed = 20;
        } else {
            speed = 10;
        }
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            pulsingColor(speed, 150, LED_Constants.Blue);
        } else {
            pulsingColor(speed, 150, LED_Constants.Red);
        }
    }

    public void runLEDS() {
        if (type.equals("normal")) {
            pulsingTele();
        } else if (type.equals("cone")) {
            flashingColor(8, LED_Constants.Yellow);
        } else if (type.equals("cube")) {
            flashingColor(8, LED_Constants.Purple);
        }
    }

    @Override
    public void periodic() {
        led.setData(ledBuffer);
    }
}