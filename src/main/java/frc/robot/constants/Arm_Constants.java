package frc.robot.constants;

public final class Arm_Constants {
    public final static int armRotatePort = 8; //filler number, change later
    public final static int armExtendPort = 9; //filler number, change later
    public final static double rotationToDegreeConversion = 275; //gear ratio for rotation moter
    public final static double armExtendSpeed = 0.2; //needs testing
    public final static double armRotateSpeed = 0.2; //needs testing
    public final static double manualExtendSpeed = 0.1;
    public final static double manualRotateSpeed = 0.1;
    
    public final static double armIntakeRotation = 0;
    public final static double topCubeRotation = 45;
    public final static double middleCubeRotation = 30;
    public final static double topConeRotation = 60;
    public final static double middleConeRotation = 50;
    public final static double hybridRotation = 20;

    public final static double armIntakeExtension = 0;
    public final static double topCubeExtension = 50;
    public final static double middleCubeExtension = 40;
    public final static double topConeExtension = 55;
    public final static double middleConeExtension = 45;
    public final static double hybridExtension = 30;

    public final static double rotationSafe = 0; //the minimum degrees at which rotation is safe
}
