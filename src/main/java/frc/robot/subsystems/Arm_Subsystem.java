// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SparkMaxWrapper;
import frc.robot.simulation.JMoneyEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm_Subsystem extends SubsystemBase {
  public static CANSparkMax armRotateMotor = new CANSparkMax(Constants.armRotatePort, MotorType.kBrushless);
  public static CANSparkMax armExtendMotor = new CANSparkMax(Constants.armExtendPort, MotorType.kBrushless);
  //public static SparkMaxWrapper armRotateMotor = new SparkMaxWrapper(Constants.armRotatePort, MotorType.kBrushless);
  //public static SparkMaxWrapper armExtendMotor = new SparkMaxWrapper(Constants.armExtendPort, MotorType.kBrushless);

  //public static RelativeEncoder rotateEncoder = armRotateMotor.getEncoder();
  //public static RelativeEncoder extendEncoder = armExtendMotor.getEncoder();
  public static JMoneyEncoder rotateEncoder = new JMoneyEncoder(armRotateMotor, 3.0);
  public static JMoneyEncoder extendEncoder = new JMoneyEncoder(armExtendMotor, 3.0);
  
  public static double desiredRotation = 0.0;
  public static double desiredExtension = 0.0;

  public static String activePreset = "None";

  //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
  //int pulsePerDegree = pulse / 360;    

  //final EncoderSim rotateEncoderSim = new EncoderSim(rotateEncoder);
  //final EncoderSim extendEncoderSim = new EncoderSim(extendEncoder);

  /** Creates a new Arm. */
  public Arm_Subsystem() {
    //possibly helpful info: 42 counts of the encoder for one rev on neos

    //delete these if it creates issues
    armRotateMotor.restoreFactoryDefaults();
    armExtendMotor.restoreFactoryDefaults();
    /**
    rotateEncoder.setPositionConversionFactor(0.0); /sets origin 
    int pulse = rotateEncoder.getCountsPerRevolution() / 4; //converts counts into pulses 
    int pulsePerDegree = pulse / 360; //figures out how many pulses per degree, so we can use that
    */
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    arcadeArm();
  }

  private void arcadeArm() {
    armRotate();
    armExtend();
    rotateEncoder.update();
    extendEncoder.update();
  }

  private void armRotate() {
    double rotatePosition = rotateEncoder.getPosition() * Constants.rotationToDegreeConversion; //gear ratio TT
    if (Math.abs(rotatePosition-desiredRotation) < 0.3) {
      armRotateMotor.set(0.0);
    }
    else if (rotatePosition < desiredRotation) {
      armRotateMotor.set(Constants.armRotateSpeed);
    } 
    else if (rotatePosition > desiredRotation) {
      armRotateMotor.set(-Constants.armRotateSpeed);
    }
  }

  private void armExtend() {
    double extendPosition = extendEncoder.getPosition() * Constants.rotationToDegreeConversion; //gear ratio TT
    if (Math.abs(extendPosition-desiredExtension) < 0.3){
      armExtendMotor.set(0.0);
    }
    else if (extendPosition < desiredExtension){
      armExtendMotor.set(Constants.armExtendSpeed);
    } 
    else if (extendPosition > desiredExtension) {
      armExtendMotor.set(-Constants.armExtendSpeed);
    }
  }

  public void moveArm(double rotation, double extension) {
    desiredRotation = rotation;
    desiredExtension = extension;
  }
}
