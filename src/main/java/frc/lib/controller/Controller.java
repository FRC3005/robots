package frc.lib.controller;

import edu.wpi.first.util.sendable.Sendable;

public interface Controller extends Sendable {

    /**
     * Set the controller target reference
     * 
     * @param reference is the controller setpoint
     */
	default void setReference(double reference) {
        setReference(reference, 0.0);
    }

    /**
     * Set the controller target reference
     * @param reference is the controller setpoint
     * @param feedforward arbitrary feedforward voltage applied after the controller
     */
	void setReference(double reference, double feedforward);

}
