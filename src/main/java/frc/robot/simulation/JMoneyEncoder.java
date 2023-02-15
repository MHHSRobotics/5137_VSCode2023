package frc.robot.simulation;

import com.revrobotics.CANSparkMax;

public class JMoneyEncoder {
    public CANSparkMax m_motor;
    private Double distance;
    private Double rotationPerUpdate;

    /**
     * The epic JMoneyEncoder that simulates an encoder measuring DISPLACEMENT of the motor.
     * This is only temporary I just wanted to do this for fun.
     *  -- JMoney
     * @param motor The motor you want to link the epic JMoneyEncoder too. ONLY CANSparkMax
     * @param timePerRotation The amount of time in seconds until the motor does a full rotation.
    */
    public JMoneyEncoder(CANSparkMax motor, Double timePerRotation) {
        m_motor = motor;
        distance = 0.0;
        rotationPerUpdate = 0.02/timePerRotation;
    }

    public void update() {
        distance += m_motor.get()*rotationPerUpdate;
    }
    
    public double getPosition() {
        return distance;
    }

    public double getDegrees() {
        return distance*360;
    }

    public void reset() {
        distance = 0.0;
    }
}
