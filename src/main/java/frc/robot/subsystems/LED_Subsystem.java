package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.constants.LED_Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.LED_Commands;

public class LED_Subsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDSim ledSim;
    private static AddressableLEDBuffer ledBuffer;
    private static int pulse = 0;
    private static String type = "None";
    private static String stage = "None";
    private static Timer time;

    //Middle led is 64

    public LED_Subsystem() {
        led = new AddressableLED(LED_Constants.Port);
        ledSim = new AddressableLEDSim(led);
        ledBuffer = new AddressableLEDBuffer(LED_Constants.Length);
        led.setLength(ledBuffer.getLength());
        time = new Timer();
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

    public void pulsingCG(double speed, double spacing) {
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

    public void allianceColor(double speed, double spacing) {
        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            pulsingColor(speed, spacing, LED_Constants.Blue);
        } else {
            pulsingColor(speed, spacing, LED_Constants.Red);
        }
    }

    public void stopLight() {
        if (stage.equals("Loaded")) {
            solidColor(LED_Constants.Red);
        } else if (stage.equals("InPos")) {
            solidColor(LED_Constants.Yellow);
        } else if (stage.equals("GO")) {
            solidColor(LED_Constants.Green);
        }
    }

    public void endGame() {
        pulsingColor(20, 25, LED_Constants.Green);
    }

    public void signal(String kType) {
        if (type.equals(kType)) {
            type = "None";
        } else {
            type = kType;
        }

        if (type.equals("Cone")) {
            solidColor(LED_Constants.Yellow);
        } else if (type.equals("Cube")) {
            solidColor(LED_Constants.Purple);
        }
    }

    public void runLEDS() {
        if (time.hasElapsed(105)) {
            if (type == "None") {
                endGame();
            }   
        } else {
            if (type == "None" && stage == "None") {
                allianceColor(20, 25);
            } else if (!(stage == "None")) {
                stopLight();
            }
        }
    }

    public void startTimer() {
        time.reset();
        time.start();
    }

    @Override
    public void periodic() {
        if (RobotContainer.shuffleboard.getLEDsEnabled()) {
            if (RobotState.isDisabled()) {
                pulsingCG(20, 25);
            } else if (RobotState.isAutonomous()) {
                allianceColor(20, 150);
            } else if (RobotState.isTeleop()) {
                runLEDS();
            }
            led.setData(ledBuffer);
        } else {
            solidColor(LED_Constants.None);
        }
    }
}