package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.constants.LED_Constants;

public class LED_Subsystem extends SubsystemBase {
    private static AddressableLED led;
    private static AddressableLEDSim ledSim;
    private static AddressableLEDBuffer ledBuffer;
    private static int pulse;
    private static int pulse2;
    private static int speed;

    public LED_Subsystem() {
        led = new AddressableLED(LED_Constants.Port);
        ledSim = new AddressableLEDSim(null);
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

    public void singleColor(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        led.setData(ledBuffer);
    }

    public void multiColors(double spacing, Color... colors) {
        double amt = colors.length; //Amount of colors
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            for (var x = 0; x < amt+1; x++) {
                if ((i % (amt*spacing)) < x*spacing && (i % (amt*spacing)) > (x-1)*spacing) {
                    ledBuffer.setLED(i, colors[x-1]);
                }
            }
        }
        led.setData(ledBuffer);
    }

    public void movingColors(double spacing, int kSpeed, Color... colors) {
        speed = kSpeed;
        double amt = colors.length; //Amount of colors
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            for (var x = 0; x < amt+1; x++) {
                if ((i % (amt*spacing)) < x*spacing && (i % (amt*spacing)) > (x-1)*spacing) {
                    ledBuffer.setLED((i+(pulse/50))%ledBuffer.getLength(), colors[x-1]);
                }
            }
        }
        led.setData(ledBuffer);
    }

    public void pulsingColors(double spacing, int kSpeed, Color mainColor, Color... colors) {
        speed = kSpeed;
        double amt = colors.length; //Amount of colors
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            for (var x = 0; x < amt+1; x++) {
                if ((i % (amt*spacing)) < x*spacing && (i % (amt*spacing)) > (x-1)*spacing) {
                    Color currentColor = colors[x-1];
                    double dif = (amt*spacing-(i%(amt*spacing)))/(amt*spacing);
                    double posDif = (i%(amt*spacing))/(amt*spacing);
                    double red = (255*currentColor.red)*dif + (255*mainColor.red)*posDif;
                    double green = (255*currentColor.green)*dif + (255*mainColor.green)*posDif;
                    double blue = (255*currentColor.blue)*dif + (255*mainColor.blue)*posDif;
                    Color finalColor = new Color((int)red, (int)green, (int)blue);
                    ledBuffer.setLED((i+(pulse/50))%ledBuffer.getLength(), finalColor);
                }
            }
        }
        led.setData(ledBuffer);
    }

    @Override
    public void periodic() {
        if (speed > 0 && ledBuffer.getLength() > 0) {
            pulse += speed;
            pulse %= 50*speed*ledBuffer.getLength();
        }
    }
}