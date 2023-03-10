package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Arm_Constants;
import frc.robot.constants.Controller_Constants.XBOX_Constants;
import frc.robot.constants.Controller_Constants.XBOX_Constants;
import frc.robot.objects.*;
import frc.robot.RobotContainer;

public class Arm_Subsystem extends SubsystemBase {
    private static SparkMaxWrapper rotateMotor;
    private static SparkMaxWrapper extendMotor;

    private static RelativeEncoder rotateEncoder; //Switch to relatives encoders once arm actually works
    private static RelativeEncoder extendEncoder;

    private static Double desiredRotation;
    private static Double desiredExtension;
    private static Double currentRotation;
    private static Double currentExtension;

    private static Boolean rotateOverride;
    private static Boolean extendOverride;

    public BooleanSupplier isFinished;
    public Consumer<Boolean> endCommand;

    //private static final Double rotateMargin = Arm_Constants.armRotateSpeed*1.5;
    //private static final Double extendMargin = Arm_Constants.armExtendSpeed*1.5;

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
        rotateMotor = new SparkMaxWrapper(Arm_Constants.armRotatePort, MotorType.kBrushless);
        extendMotor = new SparkMaxWrapper(Arm_Constants.armExtendPort, MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        extendEncoder = extendMotor.getEncoder();


        rotateEncoder.setPosition(0) /*/ (- 211.84 ))-42.7458)*/;
        extendEncoder.setPosition(0);

        //rotateMotor.burnFlash();
        //extendMotor.burnFlash();

        desiredRotation = 0.0;
        desiredExtension = 0.0; 



        isFinished = () -> {
            if (((Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError)   && (Math.abs(currentExtension-desiredExtension) < Arm_Constants.extendeMarginOfError)) || (rotateEncoder.getPosition() >= Arm_Constants.rotationIntakeSafe && RobotContainer.pneumatics_Subsystem.getIntakeEnabled())) {
                return true;
            } else {
                return false;
            }
        };

        endCommand = (finished) -> {
            if (finished) {
                stopArm(); 
            }

        };

        rotateOverride = true;
        extendOverride = false;

        this.controller = controller;
    }
 
    @Override
    public void periodic() {
        currentRotation = rotateEncoder.getPosition();
        currentExtension = -extendEncoder.getPosition();
        System.out.println("Rotate " + getRotationPosition() + "\t\t\tDesired" + desiredRotation + "\t\t\tExtention " + getExtensionPosition() + "\t\t\tDesired " + desiredExtension);//("Extent " + getExtensionPosition() + "\t\t\tDesired " + desiredExtension + "\t\t\tDiff " + Math.abs(desiredExtension - currentExtension));//"\t\t\tExtend " + getExtensionPosition());
        //System.out.println("CurExx " + currentExtension + "\t\t\tDesiredex" + desiredExtension);
        //System.out.println("diff" + (desiredExtension-currentExtension));
        
        arcadeArm();   

        if (RobotState.isEnabled()){
            if (Math.abs(currentRotation-desiredRotation) < Arm_Constants.rotateMarginOfError){
                rotateMotor.setIdleMode(IdleMode.kBrake);
                //System.out.println("it's breaking\t\t\t" + Math.abs(currentRotation-desiredRotation));
            }
            else {
                rotateMotor.setIdleMode(IdleMode.kCoast);
                //System.out.println("it's coasting\t\t\t" + Math.abs(currentRotation-desiredRotation));
            }

            if (extendMotor.getOutputCurrent() < 3.5){
                extendMotor.setIdleMode(IdleMode.kBrake);
            }
            else {
                extendMotor.setIdleMode(IdleMode.kCoast);
    
            }
        }
        else {
            rotateMotor.setIdleMode(IdleMode.kCoast);
            extendMotor.setIdleMode(IdleMode.kCoast);
        }

    
        armExtendDirection();
    }

    public void moveArm(double rotation, double extension) {
        desiredRotation = rotation;
        desiredExtension = extension;
        
        //System.out.println("/n/n/n/nMOVE ARM IS CALLED " + desiredExtension + "/n/n/n/n/n");
    }

    public void resetOverride() {
        rotateOverride = false;
        extendOverride = false;
    }

    public void stopArm() {
       desiredRotation = currentRotation;
       desiredExtension = currentExtension;
        System.out.println("stop arm");
        rotateMotor.stopMotor();
        rotateMotor.stopMotor();
    }

    public double getRotationSpeed() {
        return rotateMotor.get();
    }

    public double getExtensionSpeed() {
        return extendMotor.get();
    }

    public double getRotationPosition() {
        return (rotateEncoder.getPosition()) ; /*/ -211.84) -42.7458 /**Arm_Constants.rawToDegreeConversion */  //-48786.85844 -45.86875459; 
    }

    public double getExtensionPosition() {
        return (extendEncoder.getPosition()) ; /*/ -2237.521 +2)/*Arm_Constants.rawToInchesConversion*/
    }

    private void arcadeArm() {
        armRotate();

        if(Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 ){
            armExtendManual();
        }
        else if (!extendOverride){
            armExtendPreset();
        }
        
        extendOverride = false;
        
    }

    private void armRotate() {
        //System.out.println("arm encoder (deg)" + currentRotation);
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1 /*|| rotateOverride*/) {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            rotateOverride = true;
        } else {
            if (Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError) {
                rotateMotor.stopMotor();
            } 
            else if ((desiredRotation - currentRotation) < 0) {
                rotateMotor.set(-Arm_Constants.armRotateSpeed);
            } 
            else if ((desiredRotation - currentRotation) > 0) {
                rotateMotor.set(Arm_Constants.armRotateSpeed);
            } 
            else {
                rotateMotor.stopMotor();
            }
        }
    }

    private void armExtendManual() {
        if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 /*|| extendOverride*/) {
            if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor(); 
                    System.out.println("kill in joysticks");
                }
            extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            //System.out.println("\t\t\t\tJOYSTICKS");
            extendOverride = true;
        } 
    }
   
    private void armExtendPreset() {
        if (extendOverride){
            desiredExtension = currentExtension;
            stopArm();
        }
        else{ 
            if (Math.abs(currentExtension) > Arm_Constants.armLimit || (!armExtendDirection() && currentExtension <= 0.1)) {
                //Stops motor if extended too far or if trying to retract in too far
                extendMotor.stopMotor(); 
                //System.out.println("limit reached");
            }
            else if((currentRotation > Arm_Constants.frontExtensionSafe && currentRotation < Arm_Constants.backExtensionSafe) /*|| Math.abs(desiredExtension- currentExtension) > 97*/) {
                //Retracts extension when in danger zone for penalties
                extendMotor.set(Arm_Constants.armExtendSpeed);
                    System.out.println("retracting in zone");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor();
                    desiredExtension = currentExtension; 
                    System.out.println("kill backup");
                }
            }
            else if ((desiredExtension - currentExtension) > 0){ //Extends out if needed
                extendMotor.set(-Arm_Constants.armExtendSpeed);
                System.out.println("extending");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor(); 
                    desiredExtension = currentExtension;
                    System.out.println("kill backup 1");
                }
            }
            else if ((desiredExtension - currentExtension) < 0){ //Retracts if needed
                extendMotor.set(Arm_Constants.armExtendSpeed);  
                System.out.println("retracing");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor(); 
                    desiredExtension = currentExtension;
                    System.out.println("Kill backup 2");
                }
            }  
            else {
                extendMotor.stopMotor();
            }
        }
    }

    private boolean armExtendDirection(){
        if(desiredExtension-currentExtension > 0){
            //System.out.println("arm is extending"); 
            return true;//If the intake is extending(negative is extending)
          
        }
        else{
            //System.out.println("arm is retracting");
            return false; //If the intake is retracting
        }
    }

    private boolean armRotateDirection(){
        if(rotateMotor.get() > 0){
            return true; //If the arm is rotating towards scoring
        }
        else{
            return false; //If the arm is rotating away from scoring
        }
    }
}