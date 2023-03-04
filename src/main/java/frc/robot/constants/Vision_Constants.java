package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Vision_Constants {
    public final static double pi = Math.PI;
    public final static double nodeSpacing = Units.inchesToMeters(22);
    public final static double scoreDistance = Units.inchesToMeters(17.25); //blue line is 14.25 inches from tag, want to be another 3 inches

    public static final Transform3d robotToAR1Cam = new Transform3d(new Translation3d(-Units.inchesToMeters(0), -Units.inchesToMeters(0), -Units.inchesToMeters(0)), new Rotation3d(0, 0, 0));
    public static final Transform3d robotToAR2Cam = new Transform3d(new Translation3d(-Units.inchesToMeters(0), -Units.inchesToMeters(0), -Units.inchesToMeters(0)), new Rotation3d(0, 0, 0));

    //Red Alliance
    public final static Pose2d tag1 = new Pose2d(15.513558, 1.071626, new Rotation2d(pi));
    public final static Pose2d tag2 = new Pose2d(15.513558, 2.748026, new Rotation2d(pi));
    public final static Pose2d tag3 = new Pose2d(15.513558,4.424426, new Rotation2d(pi));
    public final static Pose2d tag4 = new Pose2d(16.178784, 6.749796, new Rotation2d(pi));

    //Blue Alliance 
    public final static Pose2d tag5 = new Pose2d(0.36195, 6.749796, new Rotation2d(0));
    public final static Pose2d tag6 = new Pose2d(1.02743, 4.424426, new Rotation2d(0));
    public final static Pose2d tag7 = new Pose2d(1.02743, 2.748026, new Rotation2d(0));
    public final static Pose2d tag8 = new Pose2d(1.02743, 1.071626, new Rotation2d(0));

    //AlignField 
    
    //Red Alliance Align Spots  
    private final static Pose2d pose1a = new Pose2d(15.513558 - scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(pi));
    private final static Pose2d pose1b = new Pose2d(15.513558 - scoreDistance, 1.071626, new Rotation2d(pi));
    private final static Pose2d pose1c = new Pose2d(15.513558 - scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(pi));
    private final static Pose2d pose2a = new Pose2d(15.513558 - scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(pi));
    private final static Pose2d pose2b = new Pose2d(15.513558 - scoreDistance, 2.748026, new Rotation2d(pi));
    private final static Pose2d pose2c = new Pose2d(15.513558 - scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(pi));
    private final static Pose2d pose3a = new Pose2d(15.513558 - scoreDistance,4.424426 - nodeSpacing, new Rotation2d(pi));
    private final static Pose2d pose3b = new Pose2d(15.513558 - scoreDistance,4.424426, new Rotation2d(pi));
    private final static Pose2d pose3c = new Pose2d(15.513558 - scoreDistance,4.424426 + nodeSpacing, new Rotation2d(pi));
    

    //Loading stations
    private final static Pose2d pose4 = new Pose2d(16.178784 - scoreDistance, 6.749796, new Rotation2d(pi));
    private final static Pose2d pose5 = new Pose2d(0.36195 + scoreDistance, 6.749796, new Rotation2d(0));

    //Blue Alliance align spots
    private final static Pose2d pose6a = new Pose2d(1.02743 + scoreDistance, 4.424426 + nodeSpacing, new Rotation2d(0));
    private final static Pose2d pose6b = new Pose2d(1.02743 + scoreDistance, 4.424426, new Rotation2d(0));
    private final static Pose2d pose6c = new Pose2d(1.02743 + scoreDistance, 4.424426 - nodeSpacing, new Rotation2d(0));
    private final static Pose2d pose7a = new Pose2d(1.02743 + scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(0));
    private final static Pose2d pose7b = new Pose2d(1.02743 + scoreDistance, 2.748026, new Rotation2d(0));
    private final static Pose2d pose7c = new Pose2d(1.02743 + scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(0));
    private final static Pose2d pose8a = new Pose2d(1.02743 + scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(0));
    private final static Pose2d pose8b = new Pose2d(1.02743 + scoreDistance, 1.071626, new Rotation2d(0));
    private final static Pose2d pose8c = new Pose2d(1.02743 + scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(0));

    public final static Pose2d[][] alignArray = {
        {pose1a, pose1b, pose1c},
        {pose2a, pose2b, pose2c},
        {pose3a, pose3b, pose3c},
        {pose4,  pose4,  pose4},
        {pose5,  pose5,  pose5}, 
        {pose6a, pose6b, pose6c},
        {pose7a, pose7b, pose7c},
        {pose8a, pose8b, pose8c}
    };
}
