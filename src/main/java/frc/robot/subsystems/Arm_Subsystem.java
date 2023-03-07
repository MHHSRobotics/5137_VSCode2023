// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.simulation.SparkMaxWrapper;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm_Subsystem extends SubsystemBase {
  CANSparkMax armRotateMotor = new CANSparkMax(Constants.armRotatePort, MotorType.kBrushless);
  CANSparkMax armExtendMotor = new CANSparkMax(Constants.armExtendPort, MotorType.kBrushless);
  //SparkMaxWrapper armRotateMotor = new SparkMaxWrapper(Constants.armRotatePort, MotorType.kBrushless);
  //SparkMaxWrapper armExtendMotor = new SparkMaxWrapper(Constants.armExtendPort, MotorType.kBrushless);
  //final EncoderSim rotateEncoderSim = new EncoderSim(rotateEncoder)
  //final EncoderSim extendEncoderSim = new EncoderSim(extendEncoder)
  private ArmFeedforward feedForward = new ArmFeedforward(Constants.kS, Constants.kG, Constants.kV, Constants.kA);
  
  public RelativeEncoder rotateEncoder = armRotateMotor.getEncoder();
  RelativeEncoder extendEncoder = armExtendMotor.getEncoder();

  public static double desiredRotation = -55;
  public static double desiredExtension = -55;

  private double rotatePosition;
  private double extendPosition;

  private PIDController rotatePID = new PIDController(Constants.aKP, Constants.aKD, Constants.aKI);
  private PIDController extendPID = new PIDController(Constants.eKP, Constants.eKD, Constants.eKI);


  private SlewRateLimiter armRotate = new SlewRateLimiter(1);
  //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
  //int pulsePerDegree = pulse / 360;    

  

  /** Creates a new Arm. */
  public Arm_Subsystem() {
    //possibly helpful info: 42 counts of the encoder for one rev on neos

    //delete these if it creates issues
    //armRotateMotor.restoreFactoryDefaults();
   // armExtendMotor.restoreFactoryDefaults();

    rotateEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);          
    //extendEncoder.setPositionConversionFactor(Constants.rotationToDegreeConversion);

    // Sets origin 
    //rotateEncoder.setPosition(0);
    //extendEncoder.setPosition(0);
    armRotateMotor.setIdleMode(IdleMode.kCoast);

  
    
    //int pulse = rotateEncoder.getCountsPerRevolution() / 4;         //converts counts into pulses 
    //int pulsePerDegree = pulse / 360;                               //figures out how many pulses per degree, so we can use that
    }

    public void setRotation(double rotationDegrees)
    {
      desiredRotation = rotationDegrees;
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      //arcadeArm();
      rotatePosition = rotateEncoder.getPosition()/200.0;
      extendPosition = extendEncoder.getPosition();

      // System.out.println("rotate encoder" + rotatePosition);
    }

    private void arcadeArm() {
       armRotate();
      armExtend();
    }

    public void moveArm(double rotation, double extension) {
      desiredRotation = rotation;
      desiredExtension = extension;
    }
   
    public void manualRotate (Joystick controller)
    {
      double speed = controller.getRawAxis(Constants.a_RXStickAxisPort);
      setRotation(speed*180);
    }

    private void armRotate() {

      //System.out.println("Desired Rotation:"+desiredRotation+"Current Rotation:"+rotatePosition);
      if(rotatePosition < desiredRotation) {
        double speed = rotatePID.calculate(rotatePosition, desiredRotation);
        armRotateMotor.setVoltage(speed + feedForward.calculate(desiredRotation, 2));
        //armRotateMotor.setVoltage(speed );

        //System.out.println("volt" + (speed + feedForward.calculate(desiredRotation, 2)));
      } 
      else if (rotatePosition > desiredRotation) {
        double speed = -rotatePID.calculate(rotatePosition, desiredRotation);
        //armRotateMotor.setVoltage(speed);
        armRotateMotor.setVoltage(speed + feedForward.calculate(desiredRotation, 2));
      }
    }

    public void armRotate (int direction){
      if (rotatePosition >= Constants.maxRotationBack && rotatePosition <= Constants.maxRotationFront) {
        armRotateMotor.set(Constants.manualRotateSpeed * direction);
      }
    }

    private void armExtend() {
      //System.out.println("Desired Extension:"+desiredExtension+"Current Extension:"+extendPosition);
      if (extendPosition < desiredExtension){
        armExtendMotor.set(extendPID.calculate(extendPosition, desiredExtension));
      } 
      else if (extendPosition > desiredExtension) {
        armExtendMotor.set(-extendPID.calculate(extendPosition, desiredExtension));
      }
    }

    public boolean armFinished(double rotation, double extention ){
      
      if ((rotatePosition-rotation < Math.abs(1)) && (extendPosition-extention < Math.abs(1.0))){
        return true;
      }
      return false;
     }

         
    //checks that arm is "above" motor or that intake is extended before moving. 
     public boolean armMovementClear(Intake_Subystem intake_Subystem){
      if (rotateEncoder.getPosition() >= Constants.intakeRotationSafe || intake_Subystem.getIntakeActive()){
        return true;
      }
      return false;
     }
    
    
  
}
