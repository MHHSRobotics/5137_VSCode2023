package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.Arm_Constants;
import frc.robot.constants.Controller_Constants.XBOX_Constants;
import frc.robot.objects.*;

public class Arm_Subsystem extends SubsystemBase {
    private static SparkMaxWrapper rotateMotor;

    private static RelativeEncoder rotateEncoder; //Switch to relatives encoders once arm actually works

    private static Double desiredRotation;
    private static Double currentRotation;

    public BooleanSupplier isFinished;
    public Consumer<Boolean> endCommand;

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
       
        rotateMotor = new SparkMaxWrapper(Arm_Constants.armRotatePort, MotorType.kBrushless); //creates motor with proper CAN id
        rotateEncoder = rotateMotor.getEncoder(); //creates encoder
        
        rotateEncoder.setPosition(Arm_Constants.startPosition); //Relative encoder start position
        currentRotation = Arm_Constants.startPosition; //Start position
        desiredRotation = Arm_Constants.startPosition; //Makes sure it matches the start position
        stopArm(); // To ensure that current rotation and desired are equal

        //Used to end the arm prests' functional commands 
        isFinished = () -> {
            if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 || Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
                return true;
            }
            else if ((Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError)){
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
        currentRotation = rotateEncoder.getPosition(); //Sets currentRotation to the current encoder reading
        arcadeArm(); //Calls method that splits up tasks for manual or preset movement
    }

    //Called by commands to set a desired rotation
    public void moveArm(double rotation) {
        desiredRotation = rotation; 
    }
 
    //Brakes the arm and makes sure it will no longer try to move anywhere
    public void stopArm() {
        desiredRotation = currentRotation;   //Ensures preset rotation will stop
        rotateMotor.setIdleMode(IdleMode.kBrake);
    }   

    //Sets the arm to coast mode - free movement not restructed
    public void coastArm()
    {
        rotateMotor.setIdleMode(IdleMode.kCoast);
    }

    //Manages braking in the arm and manual vs preset movement
    private void arcadeArm() {       
        //If the robot is not enabled then the arm can be moved by humans
        if(RobotState.isDisabled())
        {
            coastArm();
        }
        //If the joystick is being used allow the arm to rotate manually
        else if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
            coastArm();
            armRotateManual();
        }
        //If the arm needs to move more then the margin of error it will call it to move
        else if (Math.abs(desiredRotation-currentRotation) > Arm_Constants.rotateMarginOfError){
            coastArm();
            armRotatePresets();
        }
        //If the arm is not trying to move and is in the catapult stopping zone the motor will coast
        else if(currentRotation > Arm_Constants.flingCoastPosition)
        {
            coastArm();
        }
        //Otherwise will brake
        else{
            stopArm();
        }
    }

    private void armRotateManual() {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.manualRotateSpeed);
            desiredRotation = currentRotation;
    }
    

    //Will rotate to a desired position based on a set desiredrotation value. If approaching the catapult position will instead coast to ensure that the motor does not try to move into stopper 
    private void armRotatePresets() { 

        if ((desiredRotation - currentRotation) > Arm_Constants.rotateMarginOfError && currentRotation > Arm_Constants.flingCoastPosition ) {
            coastArm();
        } 
        else if ((desiredRotation - currentRotation) > Arm_Constants.rotateMarginOfError) {
            rotateMotor.set(Arm_Constants.flingSpeed);
        } 
        else if ((desiredRotation - currentRotation) <  -Arm_Constants.rotateMarginOfError) {
            rotateMotor.set(-Arm_Constants.reloadSpeed);
        } 
    }
}