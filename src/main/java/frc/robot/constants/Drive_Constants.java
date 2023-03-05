package frc.robot.constants;

import java.lang.Math.*;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.PIDConstants;

public final class Drive_Constants {
    public static final double driveSensitivity = 1.0;
    public static final double turnSensitivity = 3.0;
    
    public static final int leftFrontPort = 1;
    public static final int leftBackPort = 2;
    public static final int rightFrontPort = 3;
    public static final int rightBackPort = 4;

    //measurements / values used soley for math 
    private static final double wheelDiameter = Units.inchesToMeters(6);    
    public static final double trackWidth = 20.25;
    public static final double initialLeftDistance = 0;
    public static final double initialRightDistance = 0;

    //vision stuff
    public static final double distancePerPulse_TalonFX = wheelDiameter * Math.PI ;  

//PID
    //PID for right and left side
    public final static PIDConstants drivePIDConstants = new PIDConstants(0.05,0,0); 

    //Gains for FeedForward / Left+Right motor volts
    public final static double dKS = 1.49;  
    public final static double dKV = 1.95;
    public final static double dKA = 0.08; 

    //pid for forward speed/vision
    public final static double dKP = 0.2;
    public final static double dKD = 0.0;
    public final static double dKI = 0.0;

    //pid for rotation speed/vision
    public final static double rKP = 0.004;
    public final static double rKD = 0;
    public final static double rKI = 0; 

    //pid for charge station
    public final static double bKP = 0.01;
    public final static double bKD = 0.;
    public final static double bKI = 0.;

    

    
}
