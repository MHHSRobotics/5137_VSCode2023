package frc.robot.constants;

import frc.robot.objects.AutoData;

public class Auto_Constants {
    public static final double maxVelo = 4;
    public static final double maxAccel = 3;

    public static final int autoAmount = 11;

    //Excuse my weird formatting I made it like this so I could see what I was doing better --JMoney
    public static final AutoData left_mobility = new AutoData           ("Left", "None", true, false);
    public static final AutoData left_engage = new AutoData             ("Left", "None", false, true);
    public static final AutoData left_mobility_engage = new AutoData    ("Left", "None", true, true);
    public static final AutoData left_doubleScore = new AutoData        ("Left", "DoubleScore", true, false);
    public static final AutoData left_doubleScore_engage = new AutoData ("Left", "DoubleScore", true, true);
    public static final AutoData middle_engage = new AutoData           ("Middle", "None", false, true);
    public static final AutoData right_mobility = new AutoData          ("Right", "None", true, false);
    public static final AutoData right_engage = new AutoData            ("Right", "None", false, true);
    public static final AutoData right_mobility_engage = new AutoData   ("Right", "None", true, true);
    public static final AutoData right_doubleScore = new AutoData       ("Right", "DoubleScore", true, false);
    public static final AutoData right_doubleScore_engage = new AutoData("Right", "DoubleScore", true, true);

    public static final AutoData[] autoChoices = {
        left_mobility, left_engage, left_mobility_engage, left_doubleScore, left_doubleScore_engage,
        middle_engage,
        right_mobility, right_engage, right_mobility_engage, right_doubleScore, right_doubleScore_engage
    };
}
