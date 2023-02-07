// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import java.lang.Math;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static double pi = Math.PI;
  public final static double nodeSpacing = Units.inchesToMeters(22);
  public final static double scoreDistance = Units.inchesToMeters(36);
  
  //Driving
  public final static double driveSensitivity = 1.0;
  public final static double turnSensitivity = 3.0;
  public final static double errormargin = 0.1;

  //Controller Type "xbox" or "ps4"
  public final static String controllerType = "ps4";

  //Ports

  //Controllers
  public final static int controllerPort = 0;

  //XBOX Controller Ports
  //Need to test these, ports may have changed with the removal of buttons.
  public final static int XBOX_LXStickAxisPort = 0;
  public final static int XBOX_LYStickAxisPort = 1;
  public final static int XBOX_RXStickAxisPort = 4;
  public final static int XBOX_RYStickAxisPort = 5;

  //PS4 Controller Ports
  //Need to test these. ports may have changed with the removal of buttons.
  public final static int PS4_LXStickAxisPort = 0;
  public final static int PS4_LYStickAxisPort = 1;
  public final static int PS4_RXStickAxisPort = 2;
  public final static int PS4_RYStickAxisPort = 5;

  //DriveBase Motors
  public final static int leftTalonPort = 1;
  public final static int leftFrontVicPort = 2;
  public final static int leftBackVicPort = 3;
  public final static int rightTalonPort = 4;
  public final static int rightFrontVicPort = 5;
  public final static int rightBackVicPort = 6;

  //april tags/vision
  //change these later to put actual values when we know them
  public final static double CAMERA_HEIGHT_METERS = 1.27;
  public final static double TARGET_HEIGHT_METERS = 1.46;
  public final static double CAMERA_PITCH_RADIANS = 0;
  public final static double GOAL_RANGE_METERS = 0.5;//0.0254;//1 inch, can change later

  //pid for forward speed/vision
  public final static double dKP = 0.5;
  public final static double dKD = 0;
  public final static double dKI = 0;

  //pid for rotation speed/vision
  public final static double rKP = 0.5;
  public final static double rKD = 0;
  public final static double rKI = 0;

  //Initial robot values
  public final static Rotation2d initialGyro = new Rotation2d();
  public final static Pose2d initialPose = new Pose2d();
  public final static double initialLeftDistance = 0;
  public final static double initialRightDistance = 0;
  public final static DifferentialDriveKinematics trackWidth = new DifferentialDriveKinematics(Units.inchesToMeters(20.25));

  //TagField - tag4 & tag5 are the loading station targets

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
  public final static Pose2d pose1a = new Pose2d(15.513558 - scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose1b = new Pose2d(15.513558 - scoreDistance, 1.071626, new Rotation2d(pi));
  public final static Pose2d pose1c = new Pose2d(15.513558 - scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose2a = new Pose2d(15.513558 - scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose2b = new Pose2d(15.513558 - scoreDistance, 2.748026, new Rotation2d(pi));
  public final static Pose2d pose2c = new Pose2d(15.513558 - scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose3a = new Pose2d(15.513558 - scoreDistance,4.424426 - nodeSpacing, new Rotation2d(pi));
  public final static Pose2d pose3b = new Pose2d(15.513558 - scoreDistance,4.424426, new Rotation2d(pi));
  public final static Pose2d pose3c = new Pose2d(15.513558 - scoreDistance,4.424426 + nodeSpacing, new Rotation2d(pi));
   
  //Blue Alliance align spots
  public final static Pose2d pose6a = new Pose2d(1.02743 + scoreDistance, 4.424426 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose6b = new Pose2d(1.02743 + scoreDistance, 4.424426, new Rotation2d(0));
  public final static Pose2d pose6c = new Pose2d(1.02743 + scoreDistance, 4.424426 - nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose7a = new Pose2d(1.02743 + scoreDistance, 2.748026 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose7b = new Pose2d(1.02743 + scoreDistance, 2.748026, new Rotation2d(0));
  public final static Pose2d pose7c = new Pose2d(1.02743 + scoreDistance, 2.748026 - nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose8a = new Pose2d(1.02743 + scoreDistance, 1.071626 + nodeSpacing, new Rotation2d(0));
  public final static Pose2d pose8b = new Pose2d(1.02743 + scoreDistance, 1.071626, new Rotation2d(0));
  public final static Pose2d pose8c = new Pose2d(1.02743 + scoreDistance, 1.071626 - nodeSpacing, new Rotation2d(0));

}
