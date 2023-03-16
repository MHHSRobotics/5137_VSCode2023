package frc.robot.constants;

import frc.robot.objects.AutoData;

public class Auto_Constants {
    public static final double maxVelo = 4;
    public static final double maxAccel = 3;

    public static final int autoAmount = 11;

    //Excuse my weird formatting I made it like this so I could see what I was doing better --JMoney
    public static final AutoData left_mobility = new AutoData           ("Left", "None", true, false);
    public static final AutoData middle_engage = new AutoData           ("Middle", "None", false, true);
    public static final AutoData right_mobility = new AutoData          ("Right", "None", true, false);
    public static final AutoData middle_mobility_engage = new AutoData          ("Middle", "None", true, true);

    public static final AutoData[] autoChoices = {
        left_mobility,
        middle_engage, middle_mobility_engage,
        right_mobility,
    };
}
