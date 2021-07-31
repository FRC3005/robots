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

    public ControllerSupplier(BiConsumer<Double, Double> setReference, Consumer<SendableBuilder> init) {
        m_setReference = setReference;
        m_initSendable = init;
    }

    @Override
    public void setReference(double reference, double feedforward) {
        m_setReference.accept(reference, feedforward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        if (m_initSendable != null) {
            m_initSendable.accept(builder);
        }
    }
}
