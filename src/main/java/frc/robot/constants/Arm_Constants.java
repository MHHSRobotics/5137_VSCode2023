package frc.robot.constants;

public final class Arm_Constants {
    public final static int armRotatePort = 8; 
    public final static int armExtendPort = 9; 

    public final static double rawToDegreeConversion = 275; //gear ratio for rotation moter
    public final static double rawToInchesConversion = 2091; //each rotation is 9.4 inches 

    public final static double rotateMarginOfError = 2;

    public final static double startPosition = 0; 
    public final static double flingEndPosition = 90; //jo quinn is pulling these numbers out of the void 

    public final static double flingSpeed = 0.8; //needs testing
    public final static double reloadSpeed = 0.6; //needs testing
    //public final static double autoFling = 0; eventually create new ones for each level if needed 
    public final static double manualRotateSpeed = 1;//was 0.6
 
   
    

}