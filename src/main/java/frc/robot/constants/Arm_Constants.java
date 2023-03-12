package frc.robot.constants;

public final class Arm_Constants {
    public final static int armRotatePort = 8; 
    public final static int armExtendPort = 9; 
    public final static int armExtentionLimit = 20; 

    public final static double rotateMarginOfError = 2;
    public final static double extendeMarginOfError = 1;

    public final static int newRotationOrigin = 63;

    public final static double rawToDegreeConversion = 275; //gear ratio for rotation moter
    public final static double rawToInchesConversion = 2091; //each rotation is 9.4 inches 

    public final static double armExtendSpeed = 0.45; //needs testing
    public final static double armRotateSpeed = 0.35; //needs testing
    public final static double manualExtendSpeed = 0.4;
    public final static double manualRotateSpeed = 0.35;
    
    public final static double armIntakeRotation = 7.8545; 
    public final static double armIntakeExtension = 10.0153;

    public final static double topConeRotation = 231.0139;
    public final static double topConeExtension = 18.83027;

    public final static double middleConeRotation = 245.16278;
    public final static double middleConeExtension = 1.5699;

    public final static double topCubeRotation = 242.38911;
    public final static double topCubeExtension = 18.2655;

    public final static double middleCubeRotation = 262.92669;
    public final static double middleCubeExtension = 3.1979;

    public final static double hybridRotation = 306.8378;
    public final static double hybridExtension = 6.2465;

    public final static double rotationStartIntake = 75; //makes the intake go if it's within 0 - this range  
    
    //to ensure the motor doesn't try to drive into the bot 
    public final static double frontRotationSafe = 0; // neds to be changed 
    public final static double backRotationSafe = 307.49234; //fixed

    //to ensure the arm doesn't try to telescope to far in 
    public final static double frontExtensionSafe = 97.28;
    public final static double backExtensionSafe = 228.33;
    public static final double armRetractLimit = 0;

}
