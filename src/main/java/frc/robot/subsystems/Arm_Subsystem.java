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

    private static final Double rotateMargin = Arm_Constants.armRotateSpeed*1.5;
    private static final Double extendMargin = Arm_Constants.armExtendSpeed*1.5;

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

        desiredRotation = Arm_Constants.topCubeRotation;
        desiredExtension = Arm_Constants.topConeRotation; 



        isFinished = () -> {
            if (((Math.abs(currentRotation-desiredRotation) < rotateMargin) && (Math.abs(currentExtension-desiredExtension) < rotateMargin)) || (rotateEncoder.getPosition() >= Arm_Constants.rotationSafe && RobotContainer.pneumatics_Subsystem.getIntakeEnabled())) {
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
        arcadeArm();   

        System.out.println("Rotate " + getRotationPosition() + "\t\t\tExtend " + getExtensionPosition());

        if (RobotState.isEnabled()){
            if (rotateMotor.getOutputCurrent() < 1.5){
                rotateMotor.setIdleMode(IdleMode.kBrake);
            }
            else {
                rotateMotor.setIdleMode(IdleMode.kCoast);
            }

            if (extendMotor.getOutputCurrent() < 1.5){
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
        return (rotateEncoder.getPosition()) ; /*/ -211.84) -42.7458 /**Arm_Constants.rawToDegreeConversion */  //-48786.85844 -45.86875459; 
    }

    public double getExtensionPosition() {
        return (extendEncoder.getPosition() *-1) ; /*/ -2237.521 +2)/*Arm_Constants.rawToInchesConversion*/
    }

    private void arcadeArm() {
        armRotate();
        armExtend();
    }

    private void armRotate() {
        currentRotation = rotateEncoder.getPosition()*Arm_Constants.rawToDegreeConversion;
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1 /*|| rotateOverride*/) {
            rotateMotor.set((-controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            rotateOverride = true;
        } else {
            
            if (Math.abs(currentRotation-desiredRotation) < rotateMargin) {
                rotateMotor.set(0.0);
            } else if (Math.abs((currentRotation-desiredRotation)) < rotateMargin) {
                rotateMotor.set(-Arm_Constants.armRotateSpeed);
            } else if (Math.abs((currentRotation-desiredRotation)) > rotateMargin) {
                rotateMotor.set(Arm_Constants.armRotateSpeed);
            } else {
                rotateMotor.stopMotor();
            }
        }
    }

    private void armExtend() {
        currentExtension = extendEncoder.getPosition()*Arm_Constants.rawToDegreeConversion;
        if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 /*|| extendOverride*/) {
            extendMotor.set((controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            extendOverride = true;
        } else {
            if (currentExtension > 15 || (Math.abs(currentExtension-desiredExtension) < extendMargin)) {
                extendMotor.stopMotor();
            } 
            //if arm is within the range to get penalties 
            /*else if (((Arm_Constants.frontExtensionSafe > currentRotation) || (currentRotation > Arm_Constants.backExtensionSafe)) && currentExtension > extendMargin) {
                extendMotor.set(-Arm_Constants.armExtendSpeed);
            }*/
            else if ((currentExtension-desiredExtension) < extendMargin) { 
                extendMotor.set(-Arm_Constants.armExtendSpeed);
            } else if ((currentExtension - desiredExtension) > extendMargin) {
                extendMotor.set(Arm_Constants.armExtendSpeed);
            } else {
                extendMotor.stopMotor();
            }
        }
    }
}