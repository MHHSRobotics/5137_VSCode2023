package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.PIDConstants;

public final class Drive_Constants {
    
    //Used to scale driving and turning robot
    public static final double driveSensitivity = 1.0;
    public static final double turnSensitivity = 1;
    public final static double errormargin = 0.1; 
    
    //Motor ports for taloms
    public static final int leftFrontTalonPort = 1;
    public static final int leftBackTalonPort = 2;
    public static final int rightFrontPort = 3;
    public static final int rightBackPort = 4;

    //Measurements of robot
    public static final double wheelDiameter = Units.inchesToMeters(6);    
    public final static DifferentialDriveKinematics trackWidth = new DifferentialDriveKinematics(Units.inchesToMeters(20.25));
    public static final double initialLeftDistance = 0;
    public static final double initialRightDistance = 0;
    public static final double distancePerPulse_TalonFX = ((wheelDiameter * Math.PI) /2048.0)/9.76; //2048 is the ticks per rotation for TalonFX integrated sensor 
                                                                                                 //9.76 because the the  gear ratio 9.76:1 on the drivebase

    /*
    PID
    System designed to get slower as it gets closer to target to increase accuracy 
    kP is proportional gain, kI is integral gain, kD is derivative gain
    Before testing on robot set kI and kD to 0. Calculate kP based on desired motor speed using -- Motor Speed = kP * error(distance to setpoint)
    */
        //PID for driving in Auto 
        public final static PIDConstants drivePIDConstants = new PIDConstants(0.5,0,0);

        //Gains for FeedForward (Used in autobuilder)
        public final static double dKS = 6;//Position (Volts)
        public final static double dKV = 3;//Velocity (Volts*Seconds/Distance)
        public final static double dKA = 1.7;//Acceleration (Volts*Seonds^2/Distance) should probably be higher. 

        //pid for forward speed/vision (driving forward)
        public final static double dKP = 0.1;
        public final static double dKD = 0.0;
        public final static double dKI = 0.0;

        //pid for rotation speed/vision (rotation)
        public final static double rKP = 0.004;
        public final static double rKD = 0;
        public final static double rKI = 0; 

        //pid for charge station (balance)
        public final static double bKP = 0.0150;
        public final static double bKD = 0.;
        public final static double bKI = 0.; 
}
