package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
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

    private static final Double rotateMargin = Arm_Constants.armRotateSpeed*1.5;
    private static final Double extendMargin = Arm_Constants.armExtendSpeed*1.5;

    private final Joystick controller;

    public Arm_Subsystem(Joystick controller) {
        rotateMotor = new CANSparkMax(Arm_Constants.armRotatePort, MotorType.kBrushless);
        extendMotor = new CANSparkMax(Arm_Constants.armExtendPort, MotorType.kBrushless);

        rotateEncoder = rotateMotor.getEncoder();
        extendEncoder = extendMotor.getEncoder();

        desiredRotation = Arm_Constants.armIntakeRotation;
        desiredExtension = Arm_Constants.armIntakeExtension;

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
        return rotateEncoder.getPosition()*Arm_Constants.rawToDegreeConversion;
    }

    public double getExtensionPosition() {
        return extendEncoder.getPosition()*Arm_Constants.rawToInchesConversion;
    }

    private void arcadeArm() {
        armRotate();
        armExtend();
    }

    private void armRotate() {
        currentRotation = rotateEncoder.getPosition()*Arm_Constants.rawToDegreeConversion;
        if (Math.abs(controller.getRawAxis(XBOX_Constants.RXPort)) > 0.1 /*|| rotateOverride*/) {
            rotateMotor.set((controller.getRawAxis(XBOX_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            rotateOverride = true;
        } else {
            /*
            if (Math.abs(currentRotation-desiredRotation) < rotateMargin) {
                rotateMotor.set(0.0);
            } else if ((currentRotation-desiredRotation) < rotateMargin) {
                rotateMotor.set(Arm_Constants.armRotateSpeed);
            } else if ((currentRotation-desiredRotation) > rotateMargin) {
                rotateMotor.set(-Arm_Constants.armRotateSpeed);
            } else {*/
                rotateMotor.stopMotor();
            //}
        }
    }

    private void armExtend() {
        currentExtension = extendEncoder.getPosition()*Arm_Constants.rawToDegreeConversion;
        if (Math.abs(controller.getRawAxis(XBOX_Constants.LYPort)) > 0.1 /*|| extendOverride*/) {
            extendMotor.set((-controller.getRawAxis(XBOX_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            extendOverride = true;
        } else {
            /*if (Math.abs(currentExtension-desiredExtension) < extendMargin) {
                extendMotor.stopMotor();
            } else if ((currentExtension-desiredExtension) < extendMargin) {
                extendMotor.set(Arm_Constants.armExtendSpeed);
            } else if ((currentExtension - desiredExtension) > extendMargin) {
                extendMotor.set(-Arm_Constants.armExtendSpeed);
            } else {*/
                extendMotor.stopMotor();
            //}
        }
    }
}