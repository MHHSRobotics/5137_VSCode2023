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
    public final static PIDConstants drivePIDConstants = new PIDConstants(0.05,0,0);
    
    public final static double dKS = 1.49;
    public final static double dKV = 1.95;
    public final static double dKA = 0.08;
}
