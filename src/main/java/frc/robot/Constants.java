// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Driving
  public final static double driveSensitivity = 1.0;
  public final static double turnSensitivity = 3.0;
  public final static double errormargin = 0.1;

  //Controller Type "xbox" or "ps4"
  public final static String controllerType = "xbox";

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

  //DriveBase Motors - CAN Numbers
  public final static int leftTalonPort = 1;
  public final static int leftFrontVicPort = 3;
  public final static int leftBackVicPort = 5;
  
  public final static int rightTalonPort = 2;
  public final static int rightFrontVicPort = 4;
  public final static int rightBackVicPort = 6;
}
