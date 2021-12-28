package frc.lib.electromechanical;

import frc.lib.controller.Controller;
import frc.lib.util.SendableHelper;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class ServoMotor implements Sendable {
    private final MotorController m_motor;
    private final Controller m_VelController;
    private final Controller m_PosController;
    private final Encoder m_encoder;
    private final Gearbox m_gearbox;

    public ServoMotor(MotorController motorController, Controller VelocityController, Controller PositionController,
            Encoder encoder, Gearbox gearbox) {
        m_PosController = PositionController;
        m_VelController = VelocityController;
        m_encoder = encoder;
        m_motor = motorController;
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
     * @param velocity    Target velocity in rad/s
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
     * @param position    Target position in radians
     * 
     * @param feedforward Feedforward voltage
     */
    public void setPosition(double position, double feedforward) {
        m_PosController.setReference(position, feedforward);
    }

    /**
     * Set this servo motor to 'roll over' between min and max
     * 
     * @param min
     * @param max
     */
    public void enableContinuousRotation(double min, double max) {
    }

    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (m_VelController != null) {
            SendableHelper.addChild(builder, this, m_VelController, "velController");
        }
        
        if (m_PosController != null) {
            SendableHelper.addChild(builder, this, m_PosController, "posController");
        }

        // This tells the dashboard to be careful here!
        // Realistically need to be set enable to set gains and run things
        builder.setActuator(true);

        builder.addDoubleProperty(".velocity",
            m_encoder::getVelocity,
            null);
        builder.addDoubleProperty(".position",
            m_encoder::getPosition,
            null);
    }
}
