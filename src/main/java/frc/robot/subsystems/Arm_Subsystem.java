package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.simulation.*;
import frc.robot.constants.Arm_Constants;
import frc.robot.constants.Controller_Constants.PS4_Constants;
import frc.robot.RobotContainer;

public class Arm_Subsystem extends SubsystemBase {
    private static SparkMaxWrapper rotateMotor;
    private static SparkMaxWrapper extendMotor;

    private static JMoneyEncoder rotateEncoder; //Switch to relatives encoders once arm actually works
    private static JMoneyEncoder extendEncoder;

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

        rotateEncoder = new JMoneyEncoder(rotateMotor, 3.0);
        extendEncoder = new JMoneyEncoder(extendMotor, 3.0);

        desiredRotation = 0.0;
        desiredExtension = 0.0;

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
        if (Math.abs(controller.getRawAxis(PS4_Constants.RXPort)) > 0.1 || rotateOverride) {
            rotateMotor.set(adjust(controller.getRawAxis(PS4_Constants.RXPort))*Arm_Constants.armRotateSpeed);
            desiredRotation = currentRotation;
            rotateOverride = true;
        } else {
            if (Math.abs(currentRotation-desiredRotation) < rotateMargin) {
                rotateMotor.set(0.0);
            } else if ((currentRotation-desiredRotation) < rotateMargin) {
                rotateMotor.set(Arm_Constants.armRotateSpeed);
            } else if ((currentRotation-desiredRotation) > rotateMargin) {
                rotateMotor.set(-Arm_Constants.armRotateSpeed);
            } else {
                rotateMotor.set(0.0); //Failsafe
            }
        }
        rotateEncoder.update();
    }

    private void armExtend() {
        currentExtension = extendEncoder.getPosition()*Arm_Constants.rotationToDegreeConversion;
        if (Math.abs(controller.getRawAxis(PS4_Constants.LYPort)) > 0.1 || extendOverride) {
            extendMotor.set(adjust(-controller.getRawAxis(PS4_Constants.LYPort))*Arm_Constants.armExtendSpeed);
            desiredExtension = currentExtension;
            extendOverride = true;
        } else {
            if (Math.abs(currentExtension-desiredExtension) < extendMargin) {
                extendMotor.set(0.0);
            } else if ((currentExtension-desiredExtension) < extendMargin) {
                extendMotor.set(Arm_Constants.armExtendSpeed);
            } else if ((currentExtension - desiredExtension) > extendMargin) {
                extendMotor.set(-Arm_Constants.armExtendSpeed);
            } else {
                extendMotor.set(0.0); //Failsafe
            }
        }
        extendEncoder.update(); //Remove later
    }

    private Double adjust(Double x) {
        if (Math.abs(x) < 0.1) {return 0.0;}
        else if (Math.abs(x) > 0.9) {return Math.abs(x)/x;}
        else {return x;}
      }
}