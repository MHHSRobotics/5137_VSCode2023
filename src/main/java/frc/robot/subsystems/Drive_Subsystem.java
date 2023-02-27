package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.Drive_Constants;
import frc.robot.constants.Controller_Constants.*;

public class Drive_Subsystem extends SubsystemBase {
  private static WPI_TalonFX leftFrontMotor;
  private static WPI_TalonFX leftBackMotor;
  private static WPI_TalonFX rightFrontMotor;
  private static WPI_TalonFX rightBackMotor;

  private static MotorControllerGroup leftDrive;
  private static MotorControllerGroup rightDrive;

  public static DifferentialDrive jMoneyDrive;

  private static Double currentSpeed;
  private static Double currentRotateSpeed;
  private static Boolean arcadeDriveActive;

  private Joystick controller;

  public Drive_Subsystem(Joystick controller) {
    leftFrontMotor = new WPI_TalonFX(Drive_Constants.leftFrontPort);
    leftBackMotor = new WPI_TalonFX(Drive_Constants.leftBackPort);
    rightFrontMotor = new WPI_TalonFX(Drive_Constants.rightFrontPort);
    rightBackMotor = new WPI_TalonFX(Drive_Constants.rightBackPort);

    leftDrive = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    rightDrive = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    jMoneyDrive = new DifferentialDrive(leftDrive, rightDrive);

    currentSpeed = 0.0;
    currentRotateSpeed = 0.0;
    arcadeDriveActive = false;

    this.controller = controller;
  }

  @Override
  public void periodic() {
    if (arcadeDriveActive) {
      arcadeDrive();
    }
  }

  private void arcadeDrive() {
    Double speed = adjust(controller.getRawAxis(PS4_Constants.LYPort));
    Double rotate = adjust(controller.getRawAxis(PS4_Constants.RXPort));
    currentSpeed = speed;
    currentRotateSpeed = rotate;
    jMoneyDrive.curvatureDrive(-speed/Drive_Constants.driveSensitivity, -rotate/Drive_Constants.turnSensitivity, true);
  }

  private Double adjust(Double x) {
    if (Math.abs(x) < 0.1) {return 0.0;}
    else if (Math.abs(x) > 0.9) {return Math.abs(x)/x;}
    else {return x;}
  }

  public void drive(Double speed, Double rotate) {
    arcadeDriveActive = false;
    currentSpeed = speed;
    currentRotateSpeed = rotate;
    jMoneyDrive.curvatureDrive(speed, rotate, true);
  }

  public void enableArcadeDrive() {
    arcadeDriveActive = true;
  }

  public Double getSpeed() {
    return currentSpeed;
  }

  public Double getRotateSpeed() {
    return currentRotateSpeed;
  }

  public Boolean getArcadeDriveEnabled() {
    return arcadeDriveActive;
  }
}