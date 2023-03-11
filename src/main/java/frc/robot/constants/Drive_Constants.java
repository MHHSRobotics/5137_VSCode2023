package frc.robot.constants;

import java.lang.Math.*;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.PIDConstants;

public final class Drive_Constants {
    public static final double driveSensitivity = 1.0;
    public static final double turnSensitivity = 4.0;
    public final static double errormargin = 0.1; 
    
    public static final int leftFrontTalonPort = 1;
    public static final int leftBackTalonPort = 2;
    public static final int rightFrontPort = 3;
    public static final int rightBackPort = 4;

    //measurements / values used soley for math 
    public static final double wheelDiameter = Units.inchesToMeters(6);    
    public final static DifferentialDriveKinematics trackWidth = new DifferentialDriveKinematics(Units.inchesToMeters(20.25));
    public static final double initialLeftDistance = 0;
    public static final double initialRightDistance = 0;

    //vision stuff
    public static final double distancePerPulse_TalonFX = (wheelDiameter * Math.PI) / 2048.0/10; //2048 is the ticks per rotation for TalonFX, /10 because the the selected sensor position is *10. 

//PID
    //PID for right and left side
    public final static PIDConstants drivePIDConstants = new PIDConstants(0.5,0,0);

    //Gains for FeedForward / Left+Right motor volts
    public final static double dKS = .55;
    public final static double dKV = 2.6;
    public final static double dKA = .3;

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
