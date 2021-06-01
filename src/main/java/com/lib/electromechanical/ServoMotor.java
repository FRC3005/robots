package com.lib.electromechanical;

public class ServoMotor {
    private final Motor m_motor;
    private final Controller m_VelController;
    private final Controller m_PosController;
    private final Encoder m_encoder;
    private final Gearbox m_gearbox;
    
    public ServoMotor(
        Motor motor,
        Controller VelocityController,
        Controller PositionController,
        Encoder encoder,
        Gearbox gearbox) {
            m_PosController = PositionController;
            m_VelController = VelocityController;
            m_encoder = encoder;
            m_motor = motor;
            m_gearbox = gearbox;
    }

    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    public double getPosition() {
        return m_encoder.getPosition();
    }

    public void setVelocity(double velocity) {
        m_VelController.setReference(velocity);
    }

    public void setPosition(double position) {
        m_PosController.setReference(positon);
    }
}
