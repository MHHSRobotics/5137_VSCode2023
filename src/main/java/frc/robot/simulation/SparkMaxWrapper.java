package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.RobotController;

public class SparkMaxWrapper extends CANSparkMax {
    private SimDevice simSparkMax;
    private SimDouble simSpeed;
    private Direction simDirection;

    public SparkMaxWrapper(int deviceID, MotorType type) {
        super(deviceID,type);
        simSparkMax = SimDevice.create("SparkMax", deviceID);
        simDirection = Direction.kInput;
        if (simSparkMax != null) {
            simSpeed = simSparkMax.createDouble("Speed", simDirection, 0.0);
        }
    }

    @Override
    public double get(){
        if (simSparkMax != null){
            return simSpeed.get();
        }
        return super.get();
    }

    @Override
    public void set(double speed){
        if (simSparkMax != null){
            simSpeed.set(speed);
        } else {
            super.set(speed);
        }
    }

    @Override
    public void setVoltage(double outputVolts) { //For simulation purposes, we are expecting that the battery voltage stays constant.
        if (simSparkMax != null){
            set(outputVolts / RobotController.getBatteryVoltage());
        } else {
            super.setVoltage(outputVolts);
        }
    }
}