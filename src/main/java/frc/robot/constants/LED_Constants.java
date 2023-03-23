package frc.robot.constants;

import edu.wpi.first.wpilibj.util.Color;

public class LED_Constants {

    public static final int Port = 9; //PWM PORT of LED Strip - CAN ONLY PLUG IN 1 LED DATA CABLE TO ROBORIO FOR LEDS (USE SPLITTER FOR TWO)
    public static final int Length = 150; //Length of LED Sequence

    public static final Color None = new Color(0, 0, 0);
    public static final Color Red = new Color(200, 0, 0);
    public static final Color Orange = new Color(150, 50, 0);
    public static final Color Yellow = new Color(150, 100, 0);
    public static final Color Green = new Color(0, 200, 0);
    public static final Color Cyan = new Color(0, 200, 200);
    public static final Color Blue = new Color(0, 0, 200);
    public static final Color Purple = new Color(200, 0, 200);
    public static final Color Pink = new Color(255, 0, 100);
    public static final Color Gold = new Color(100, 40, 0);
}