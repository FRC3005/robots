package com.lib.util;

import java.util.HashSet;
import java.util.function.BooleanSupplier;

public class HealthMonitor {
    private static HealthMonitor m_instance = new HealthMonitor();

    private HealthMonitor() { }

    class MonitoredElement {
        protected BooleanSupplier m_monitor;
        protected BooleanSupplier m_reinit;
        protected int m_retries = 3;
        protected int m_errorCnt = 0;

        protected boolean m_hasError = true;
        public MonitoredElement(BooleanSupplier monitor, BooleanSupplier reinit) {
            m_monitor = monitor;
            m_reinit = reinit;
        }

        public boolean hasError() {
            return m_hasError;
        }

        public void maxRetries(int retries) {
            m_retries = retries;
        }
    }

    private HashSet<MonitoredElement> m_elements;
    
    public static MonitoredElement monitor(BooleanSupplier monitorFunction, BooleanSupplier reinitFunction) {
        MonitoredElement el = m_instance.new MonitoredElement(monitorFunction, reinitFunction);
        m_instance.m_elements.add(el);
        return el;
    }
    
    public void run() {
        for (MonitoredElement el : m_elements) {
            if (el.m_monitor.getAsBoolean()) {
                // TODO: Log error occured
                if (el.m_errorCnt < el.m_retries) {
                    el.m_hasError = el.m_reinit.getAsBoolean();
                    el.m_errorCnt++;
                } else {
                    el.m_hasError = true;
                }
            } else {
                el.m_errorCnt--;
            }
        }
    }
}
