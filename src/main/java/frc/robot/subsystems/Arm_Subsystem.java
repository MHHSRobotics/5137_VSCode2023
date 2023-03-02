package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.simulation.*;
import frc.robot.constants.Arm_Constants;

public class Arm_Subsystem extends SubsystemBase {
    private static SparkMaxWrapper rotateMotor;
    private static SparkMaxWrapper extendMotor;

    private static JMoneyEncoder rotateEncoder; //Switch to relatives encoders once arm actually works
    private static JMoneyEncoder extendEncoder;

    private static Double desiredRotation;
    private static Double desiredExtension;
    private static Double currentRotation;
    private static Double currentExtension;

    public BooleanSupplier isFinished;
    public Consumer<Boolean> endCommand;

    private static final Double rotateMargin = Arm_Constants.armRotateSpeed*1.5;
    private static final Double extendMargin = Arm_Constants.armExtendSpeed*1.5;

    public Arm_Subsystem() {
        rotateMotor = new SparkMaxWrapper(Arm_Constants.armRotatePort, MotorType.kBrushless);
        extendMotor = new SparkMaxWrapper(Arm_Constants.armExtendPort, MotorType.kBrushless);

        rotateEncoder = new JMoneyEncoder(rotateMotor, 3.0);
        extendEncoder = new JMoneyEncoder(extendMotor, 3.0);

        desiredRotation = 0.0;
        desiredExtension = 0.0;

        isFinished = () -> {
            if (Math.abs(currentRotation-desiredRotation) < rotateMargin) {
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
    }

    @Override
    public void periodic() {
        arcadeArm();
    }

    public void moveArm(Double rotation, Double extension) {
        desiredRotation = rotation;
        desiredExtension = extension;
    }

    public void stopArm() {
        desiredRotation = currentRotation;
        desiredExtension = currentExtension;
        rotateMotor.set(0.0);
        rotateMotor.set(0.0);
    }

    public double getRotationSpeed() {
        return rotateMotor.get();
    }

    public double getExtensionSpeed() {
        return extendMotor.get();
    }

    public double getRotationPosition() {
        return rotateEncoder.getPosition()*Arm_Constants.rotationToDegreeConversion;
    }

    public double getExtensionPosition() {
        return extendEncoder.getPosition()*Arm_Constants.rotationToDegreeConversion;
    }

    private void arcadeArm() {
        armRotate();
        armExtend();
    }

    private void armRotate() {
        currentRotation = rotateEncoder.getPosition()*Arm_Constants.rotationToDegreeConversion;
        if (Math.abs(currentRotation-desiredRotation) < rotateMargin) {
            rotateMotor.set(0.0);
        } else if ((currentRotation-desiredRotation) < rotateMargin) {
            rotateMotor.set(Arm_Constants.armRotateSpeed);
        } else if ((currentRotation - desiredRotation) > rotateMargin) {
            rotateMotor.set(-Arm_Constants.armRotateSpeed);
        } else {
            rotateMotor.set(0.0); //Failsafe
        }
        rotateEncoder.update(); //Remove later
    }

    private void armExtend() {
        currentExtension = extendEncoder.getPosition()*Arm_Constants.rotationToDegreeConversion;
        if (Math.abs(currentExtension-desiredExtension) < extendMargin) {
            extendMotor.set(0.0);
        } else if ((currentExtension-desiredExtension) < extendMargin) {
            extendMotor.set(Arm_Constants.armExtendSpeed);
        } else if ((currentExtension - desiredExtension) > extendMargin) {
            extendMotor.set(-Arm_Constants.armExtendSpeed);
        } else {
            extendMotor.set(0.0); //Failsafe
        }
        extendEncoder.update(); //Remove later
    }
}