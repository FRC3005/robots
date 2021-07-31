package com.lib.controller;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class ControllerSupplier implements Controller {
    BiConsumer<Double, Double> m_setReference;
    Consumer<SendableBuilder> m_initSendable = null;

    public ControllerSupplier(BiConsumer<Double, Double> setReference) {
        m_setReference = setReference;
    }

    @Override
    public void setReference(double reference, double feedforward) {
        m_setReference.accept(reference, feedforward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ControllerSup");
        if (m_initSendable != null) {
            m_initSendable.accept(builder);
        }

        builder.addStringProperty(".name",
            () -> this.getClass().getName(),
            null);

        builder.addStringProperty("test123", () -> "this is a test string", null);
    }
    
}
