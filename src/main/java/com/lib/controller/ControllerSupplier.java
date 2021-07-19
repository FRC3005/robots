package com.lib.controller;

import java.util.function.BiConsumer;

public class ControllerSupplier implements Controller {
    BiConsumer<Double, Double> m_setReference;
    public ControllerSupplier(BiConsumer<Double, Double> setReference) {
        m_setReference = setReference;
    }

    @Override
    public void setReference(double reference, double feedforward) {
        m_setReference.accept(reference, feedforward);
    }
    
}
