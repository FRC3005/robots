package com.lib.electromechanical;

import com.lib.controller.Controller;

public class ServoMotor {
    private final MotorController m_motor;
    private final Controller m_VelController;
    private final Controller m_PosController;
    private final Encoder m_encoder;
    private final Gearbox m_gearbox;
    
    public ServoMotor(
        MotorController motorContorller,
        Controller VelocityController,
        Controller PositionController,
        Encoder encoder,
        Gearbox gearbox) {
            m_PosController = PositionController;
            m_VelController = VelocityController;
            m_encoder = encoder;
            m_motor = motorContorller;
            m_gearbox = gearbox;
    }

    /**
     * Get the velocity of the servo motor
     * 
     * @return Velocity in rad/s
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * Get the position of the servo motor
     * 
     * @return Position in radians
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * Set the target velocity of the servo motor.
     * 
     * @param velocity Target velocity in rad/s
     */
    public void setVelocity(double velocity) {
        m_VelController.setReference(velocity);
    }

    /**
     * Set the target velocity of the servo motor.
     * 
     * @param velocity Target velocity in rad/s
     * 
     * @param feedforward Feedforward voltage
     */
    public void setVelocity(double velocity, double feedforward) {
        m_VelController.setReference(velocity, feedforward);
    }

    /**
     * Set the target position of the servo motor.
     * 
     * @param position Target position in radians
     */
    public void setPosition(double position) {
        m_PosController.setReference(position);
    }

    /**
     * Set the target position of the servo motor.
     * 
     * @param position Target position in radians
     * 
     * @param feedforward Feedforward voltage
     */
    public void setPosition(double position, double feedforward) {
        m_PosController.setReference(position, feedforward);
    }

    /**
     * 
     * @param min
     * @param max
     */
    public void enableContinuousRotation(double min, double max) {
    }

	public void resetEncoder() {
	}
}
