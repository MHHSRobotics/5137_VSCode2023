package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.ejml.ops.SortCoupledArray_F32;

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

    private static RelativeEncoder rotateEncoder; //Switch to relatives encoders once arm actually works

    private static Double desiredRotation;
    private static Double currentRotation;

    public BooleanSupplier isFinished;
    public Consumer<Boolean> endCommand;

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
        rotateMotor = new SparkMaxWrapper(Arm_Constants.armRotatePort, MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        rotateEncoder.setPosition(Arm_Constants.startPosition); //MEANS WE NEED TO START IN INTAKE POSITION

        currentRotation = Arm_Constants.startPosition; //fixes null point error 
        desiredRotation = Arm_Constants.startPosition; //IF NOT THE SAME AS SETPOSITION WILL MOVE TO THIS AS SOON AS ROBOT IS ENABLES
        
        stopArm();

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
        currentRotation = rotateEncoder.getPosition();
        arcadeArm(); //keeping just for overiding with joysticks 
        if(Math.abs(desiredRotation-currentRotation) > Arm_Constants.rotateMarginOfError || Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1)
        {
         rotateMotor.setIdleMode(IdleMode.kCoast);
        }
        else
        {
            rotateMotor.setIdleMode(IdleMode.kBrake);  
        }
    }

        

    public void moveArm(double rotation) {
        desiredRotation = rotation;
    }
 
    
    public void stopArm() {
        desiredRotation = currentRotation;   //Ensures preset rotation will stop
        rotateMotor.setIdleMode(IdleMode.kBrake);
    }

   


    private void arcadeArm() {       
        //Allows joystick to override preset 
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1){
            armRotateManual();
        }
        else{
            armRotatePresets();
        }
    }

    private void armRotateManual() {

        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1) {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.manualRotateSpeed);
            desiredRotation = currentRotation;
        }
        else{
            stopArm();
        }
       
    }
    
    private void armRotatePresets() { 

        if (Math.abs(desiredRotation - currentRotation) < Arm_Constants.rotateMarginOfError) {
            stopArm();
        } 
        else if ((desiredRotation - currentRotation) > Arm_Constants.rotateMarginOfError) {
            rotateMotor.set(Arm_Constants.flingSpeed);
        } 
        else if ((desiredRotation - currentRotation) <  -Arm_Constants.rotateMarginOfError) {
            rotateMotor.set(-Arm_Constants.reloadSpeed);
        } 
        else {
            stopArm();
        }
    }

    

  

}