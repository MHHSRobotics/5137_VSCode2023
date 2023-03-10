package frc.robot.constants;

public final class Arm_Constants {
    public final static int armRotatePort = 8; 
    public final static int armExtendPort = 9; 
    public final static int armLimit = 20;

    public final static double rotateMarginOfError = 2;
    public final static double extendeMarginOfError = 0.25;

    public final static int newRotationOrigin = 63;

    public final static double rawToDegreeConversion = 275; //gear ratio for rotation moter
    public final static double rawToInchesConversion = 2091; //each rotation is 9.4 inches 

    public final static double armExtendSpeed = 0.35/2; //needs testing
    public final static double armRotateSpeed = 0.30/2; //needs testing
    public final static double manualExtendSpeed = 0.1;
    public final static double manualRotateSpeed = 0.1;
    
    public final static double armIntakeRotation = 4; //-59 + newRotationOrigin;
    public final static double armIntakeExtension = 8.75;

    public final static double topConeRotation = 173.766 + newRotationOrigin;
    public final static double topConeExtension = 17.22;

    public final static double middleConeRotation = 192.29 + newRotationOrigin;
    public final static double middleConeExtension = 0.66;

    public final static double topCubeRotation = 188.578 + newRotationOrigin;
    public final static double topCubeExtension = 17.481;

    public final static double middleCubeRotation = 211.787 + newRotationOrigin;
    public final static double middleCubeExtension = 5.419;

    public final static double hybridRotation = 250.755 + newRotationOrigin;
    public final static double hybridExtension = 15.099;

    public final static double rotationIntakeSafe = -20 + newRotationOrigin; //the minimum degrees at which rotation is safe 
    public final static double frontRotationSafe = 0; // neds to be changed 
    public final static double backRotationSafe = 250.755 + newRotationOrigin;
    public final static double frontExtensionSafe = 97.28;
    public final static double backExtensionSafe = 205.33;
    public static final double armRetractLimit = 0;

}
