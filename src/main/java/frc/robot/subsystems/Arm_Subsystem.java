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
    private static CANSparkMax rotateMotor;
    private static CANSparkMax extendMotor;

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

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
        rotateMotor = new CANSparkMax(Arm_Constants.armRotatePort, MotorType.kBrushless);
        extendMotor = new CANSparkMax(Arm_Constants.armExtendPort, MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        extendEncoder = extendMotor.getEncoder();


        rotateEncoder.setPosition(Arm_Constants.armIntakeRotation);//Arm_Constants.armIntakeRotation)) /*/ (- 211.84 ))-42.7458)*/;
        extendEncoder.setPosition(-Arm_Constants.armIntakeExtension);//Arm_Constants.armIntakeExtension);

        //rotateMotor.burnFlash();
        //extendMotor.burnFlash();

        desiredRotation = Arm_Constants.armIntakeRotation;//getRotationPosition();
        desiredExtension = Arm_Constants.armIntakeExtension;// getExtensionPosition(); 



        isFinished = () -> {
            if (extendOverride || rotateOverride){
                return true;
            }
            else if (((Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError)  && (Math.abs(currentExtension-desiredExtension) < Arm_Constants.extendeMarginOfError)) || (rotateEncoder.getPosition() > Arm_Constants.rotationStartIntake) ) { /* >= Arm_Constants.rotationIntakeSafe && RobotContainer.pneumatics_Subsystem.getIntakeEnabled())*/ 
                return true;
            }
            else {
                return false;
            }
        };

        endCommand = (finished) -> {
            if (finished) {
                stopArm(); 
            }

        };

        rotateOverride = false;
        extendOverride = false;

        this.controller = controller;
    }
 
    @Override
    public void periodic() {
        currentRotation = rotateEncoder.getPosition();
        currentExtension = -extendEncoder.getPosition(); //Negative because motor is inverted
        arcadeArm(); //Method called to move arm with presets or manually
        System.out.println("cRt " + currentRotation + " dRt " + desiredRotation + " cEx " + currentExtension + " dEx " + desiredExtension);

        if(Math.abs(desiredRotation-currentRotation) > 5 && currentRotation < Arm_Constants.rotationStartIntake ){
            RobotContainer.intake_Commands.justExtend(); //If the arm is set to move significantly and is near intake, intake drops
        }
        else if((Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1) && (currentRotation < Arm_Constants.rotationStartIntake))
        {
            RobotContainer.intake_Commands.justExtend(); //If the arm is being moved manually and is near intake, intake dropped
        }
        else if (!Intake_Subystem.intakeOveride) {
            RobotContainer.intake_Commands.justRetract(); //If the intake is not being called elsewhere, retract in
        }
        

        if (RobotState.isEnabled()){
            if (Math.abs(currentRotation-desiredRotation) < Arm_Constants.rotateMarginOfError){
                rotateMotor.setIdleMode(IdleMode.kBrake); //Brakes the arm when close enough to its desiredRotation
            }
            else {
                rotateMotor.setIdleMode(IdleMode.kCoast);
            }

            if (Math.abs(desiredExtension-currentExtension) < Arm_Constants.extendeMarginOfError){
                extendMotor.setIdleMode(IdleMode.kBrake);//Brakes the arm when close enough to its desiredExtension
            }
            else {
                extendMotor.setIdleMode(IdleMode.kCoast);
    
            }
        }
        else {
            rotateMotor.setIdleMode(IdleMode.kCoast);
            extendMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void moveArm(double rotation, double extension) {
        desiredRotation = rotation;
        desiredExtension = extension;
    }
 
    public void resetOverride() {
        rotateOverride = false;
        extendOverride = false;
    }
    
    public void stopArm() {
       desiredRotation = currentRotation;
       desiredExtension = currentExtension;
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
        return (currentRotation) ; /*/ -211.84) -42.7458 /**Arm_Constants.rawToDegreeConversion */  //-48786.85844 -45.86875459; 
    }

    public double getExtensionPosition() {
        return (currentExtension) ; /*/ -2237.521 +2)/*Arm_Constants.rawToInchesConversion*/
    }

    private void arcadeArm() {
        if(Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 ){
            armExtendManual();
        }
        else if (!extendOverride){
            armExtendPreset();
        }
        else{
            extendOverride = false;
        }

        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
            armRotateManual();
        }
        else if (!rotateOverride){
            armRotatePresets();
        }
        else{
            rotateOverride = false;
        }
        
        
    }

    private void armRotateManual() {
        rotateOverride = true;

        //System.out.println("arm encoder (deg)" + currentRotation);
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1) {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            System.out.println("supposed to be true");
        }
       
    }
    
    private void armRotatePresets() {
        if (rotateOverride){ 
            desiredRotation = currentRotation;
            stopArm();
        }
        else{ 
            if (Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError) {
                rotateMotor.stopMotor();
            } 
            else if ((desiredRotation - currentRotation) < 0) {
                //rotateMotor.set(-Arm_Constants.armRotateSpeed);
            } 
            else if ((desiredRotation - currentRotation) > 0) {
                //rotateMotor.set(Arm_Constants.armRotateSpeed);
            } 
            else {
                rotateMotor.stopMotor();
            }
        }
    }

    private void armExtendManual() {
        extendOverride = true;

        if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1) {
            if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                   if (armExtendDirection()) { 
                        extendMotor.stopMotor(); 
                   }
                   else if (!armExtendDirection()){
                    extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
                    desiredExtension = currentExtension;
                    extendOverride = true;
                   }
                    //System.out.println("kill in joysticks");
            }
            else {
            extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            //System.out.println("\t\t\t\tJOYSTICKS");
            extendOverride = true;
            }
            
        } 
    }
   
    private void armExtendPreset() {
        if (extendOverride){
            desiredExtension = currentExtension;
            stopArm();
        }
        else{ 
            if (Math.abs(currentExtension) > Arm_Constants.armLimit || (!armExtendDirection() && currentExtension <= 0.1) || Math.abs((desiredExtension -  currentExtension)) < Arm_Constants.extendeMarginOfError) {
                //Stops motor if extended too far or if trying to retract in too far
                extendMotor.stopMotor(); 
                //System.out.println("limit reached");
            }
            else if((currentRotation > Arm_Constants.frontExtensionSafe && currentRotation < Arm_Constants.backExtensionSafe) || Math.abs(desiredRotation- currentRotation) > 97) {
                //Retracts extension when in danger zone for penalties
                extendMotor.set(Arm_Constants.armExtendSpeed);
                    //System.out.println("retracting in zone");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor();
                    desiredExtension = currentExtension; 
                    //System.out.println("kill backup");
                }
            }
            else if ((desiredExtension - currentExtension) > 0){ //Extends out if needed
                extendMotor.set(-Arm_Constants.armExtendSpeed);
                //System.out.println("extending");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor(); 
                    desiredExtension = currentExtension;
                    //System.out.println("kill backup 1");
                }
            }
            else if ((desiredExtension - currentExtension) < 0){ //Retracts if needed
                extendMotor.set(Arm_Constants.armExtendSpeed);  
                //System.out.println("retracing");
                if ( currentExtension > Arm_Constants.armLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                    extendMotor.stopMotor(); 
                    desiredExtension = currentExtension;
                    //System.out.println("Kill backup 2");
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