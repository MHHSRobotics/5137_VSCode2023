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

    //private static Boolean rotateOverride;
    //private static Boolean extendOverride;

    public BooleanSupplier isFinished;
    public Consumer<Boolean> endCommand;

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
        rotateMotor = new SparkMaxWrapper(Arm_Constants.armRotatePort, MotorType.kBrushless);
        extendMotor = new SparkMaxWrapper(Arm_Constants.armExtendPort, MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        extendEncoder = extendMotor.getEncoder();


        rotateEncoder.setPosition(Arm_Constants.armIntakeRotation); //MEANS WE NEED TO START IN INTAKE POSITION
        extendEncoder.setPosition(-Arm_Constants.armIntakeExtension); //IF WE CHANGE THIS CHANGE DESRIRED INSTANTIATION  

        //rotateMotor.burnFlash();
        //extendMotor.burnFlash();

        currentRotation = Arm_Constants.armIntakeRotation; //fixes null point error 
        currentExtension = Arm_Constants.armIntakeExtension;

        desiredRotation = Arm_Constants.armIntakeRotation; //IF NOT THE SAME AS SETPOSITION WILL MOVE TO THIS AS SOON AS ROBOT IS ENABLES
        desiredExtension = Arm_Constants.armIntakeExtension; //arm will attempt to move to this position (if not already in it) when enabled 


        //rotateOverride = false;
        //extendOverride = false;


        //Used to end the arm prests' functional commands 
        isFinished = () -> {
            if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 || Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
                return true;
            }
            else if (((Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError)  && (Math.abs(currentExtension-desiredExtension) < Arm_Constants.extendeMarginOfError))/*  || (rotateEncoder.getPosition() > Arm_Constants.rotationStartIntake)*/ ) { /* >= Arm_Constants.rotationIntakeSafe && RobotContainer.pneumatics_Subsystem.getIntakeEnabled())*/ 
                return true;
            }
            else {
                return false;
            }
        };

        //runs when arm preset commands are ending 
        endCommand = (finished) -> {
            if (finished) {
                stopArm(); //sets desired = current so movement stops 
            }

        };

        this.controller = controller;
    }
 
    @Override
    public void periodic() {
        currentRotation = rotateEncoder.getPosition();
        currentExtension = -extendEncoder.getPosition(); //Negative because motor is inverted

        arcadeArm(); //Method called to move arm with presets or manually

        System.out.println("cRt " + currentRotation + " dRt " + desiredRotation + " cEx " + currentExtension + " dEx " + desiredExtension);

        if(Math.abs(desiredRotation-currentRotation) > 5 && currentRotation < Arm_Constants.rotationStartIntake ){
            //.schedule() is needed when commands aren't being button binded
            RobotContainer.intake_Commands.justExtend().schedule(); //If the arm is preset to move significantly (not jittering) and is near intake, intake drops
            //System.out.println("intake should be extended");
        }
        else if((Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1) && (currentRotation < Arm_Constants.rotationStartIntake)){
            RobotContainer.intake_Commands.justExtend().schedule(); //If the arm is being moved manually and is near intake, intake dropped
            //System.out.println("intake should be extended 2.0");
        }
        else if (Math.abs(RobotContainer.driverController.getRawAxis(XBOX_Constants.RTPort)) < 0.1 && Math.abs(RobotContainer.driverController.getRawAxis(XBOX_Constants.LTPort)) < 0.1) { //allows override from the driver controler 
            RobotContainer.intake_Commands.justRetract().schedule(); //If the intake is not being called elsewhere, retract in
            //System.out.println("intake should retracted");
        }
        

        //Keeps arm in place when scoring, allows us to manhandle during testing 
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
 

    //called in preset commands 
    public void resetOverride() {
        //rotateOverride = false;
        //extendOverride = false;
    }
    
    public void stopArm() {
       desiredRotation = currentRotation;   //Ensures preset rotation/extention will stop
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
        return (currentRotation); 
    }

    public double getExtensionPosition() {
        return (currentExtension); 
    }

    private void arcadeArm() {

        //Allows joystick to override preset 
        if(Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 ){
            armExtendManual();
        }
        else{
            armExtendPreset();
        }
    
        //Allows joystick to override preset 
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
            armRotateManual();
        }
        else{
            armRotatePresets();
        }
    }

    private void armRotateManual() {
        //rotateOverride = true;

        //System.out.println("arm encoder (deg)" + currentRotation);
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1) {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            System.out.println("supposed to be true");
        }
       
    }
    
    private void armRotatePresets() { 
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

    private void armExtendManual() {

        if (controller.getRawAxis(XBOX_Constants.LYPort) > 0.1) {
            if ( currentExtension > Arm_Constants.armExtentionLimit){ //please don't delete these, sometimes the limit above doesn't always catch it
                extendMotor.stopMotor(); 
            }
            else{
                extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
                desiredExtension = currentExtension;
                //System.out.println("\t\t\t\tJOYSTICKS");
                }
        }
        else if (controller.getRawAxis(XBOX_Constants.LYPort) < -0.1){
            extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            //System.out.println("\t\t\t\tJOYSTICKS");
        }
        
    }

   
    private void armExtendPreset() {
        if (Math.abs(currentExtension) > Arm_Constants.armExtentionLimit || (!armExtendDirection() && currentExtension <= 1.0)) {
            //Stops motor if extended too far or if trying to retract in too far
            extendMotor.stopMotor(); 
            //System.out.println("limit reached");
        }
        else if((currentRotation > Arm_Constants.frontExtensionSafe && currentRotation < Arm_Constants.backExtensionSafe) || Math.abs(desiredRotation- currentRotation) > 97) {
            //Retracts extension when in danger zone for penalties
            //should not extend untill past danger zone
            extendMotor.set(Arm_Constants.armExtendSpeed);
                //System.out.println("retracting in zone");
            extendLimitFailSafe();  //please don't delete these, sometimes the limit above doesn't always catch it

        }
        else if (Math.abs((desiredExtension -  currentExtension)) < Arm_Constants.extendeMarginOfError){
            //Stops it it's within our margin of error 
            //Is seperate so it doesn't interfere with danger zone retraction (otherwise will at the desired not at 0)
            extendMotor.stopMotor();
        }
        else if ((desiredExtension - currentExtension) > 0){ //Extends out if needed
            extendMotor.set(-Arm_Constants.armExtendSpeed);
            //System.out.println("extending");
            extendLimitFailSafe();  //please don't delete these, sometimes the limit above doesn't always catch it
        }
        else if ((desiredExtension - currentExtension) < 0){ //Retracts if needed
            extendMotor.set(Arm_Constants.armExtendSpeed);  
            //System.out.println("retracing");
            extendLimitFailSafe();  //please don't delete these, sometimes the limit above doesn't always catch it
        }  
        else {
            extendMotor.stopMotor(); //failsafe, ensures motor stops incase "if ()" was passed over but difference is within margin of error
        }
        
    }


    //called during every preset to ensure we're not overextending 
    private void extendLimitFailSafe(){
        if ( currentExtension > Arm_Constants.armExtentionLimit){ 
            extendMotor.stopMotor(); 
            desiredExtension = currentExtension;
            //System.out.println("Kill backup");
        }
    }


    //true = extending -- false = retracting
    private boolean armExtendDirection(){
        if(desiredExtension-currentExtension > 0){
            //System.out.println("arm is extending"); 
            return true;//(negative is extending)
          
        }
        else{
            //System.out.println("arm is retracting");
            return false; 
        }
    }


    //true = rotating back -- false = rotating towards intake (forward)
    private boolean armRotateDirection(){
        if(rotateMotor.get() > 0){
            return true; 
        }
        else{
            return false; 
        }
    }
}