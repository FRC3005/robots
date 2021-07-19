package com.lib.electromechanical;

public interface Encoder {
    public double getVelocity();
    public double getPosition();
    public void setPosition(double position);
}
