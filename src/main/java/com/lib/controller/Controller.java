package com.lib.controller;

public interface Controller {

    /**
     * Set the controller target reference
     * 
     * @param velocity
     */
	void setReference(double velocity);

	void setReference(double velocity, double feedforward);

}
