package com.lib.vendor.motorcontroller;

import java.util.function.Function;

import com.lib.controller.PIDGains;
import com.lib.controller.Controller;
import com.lib.controller.ControllerSupplier;
import com.lib.electromechanical.Encoder;
import com.lib.electromechanical.EncoderSupplier;
import com.lib.electromechanical.MotorController;
import com.lib.util.HealthMonitor;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.FaultID;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class SparkMax implements MotorController {
    /**
     * Monitor the Spark Max to check for reset. This is used by the health monitor
     * to automatically re-initialize the spark max in case of reboot.
     * 
     * @param sm Spark Max object to monitor
     * @return True if the device has reset
     */
    private static boolean sparkmaxMonitorFunction(CANSparkMax sm) {
        return sm.getStickyFault(FaultID.kHasReset);
    } 

    public SparkMax(CANSparkMax sparkMax, Function<CANSparkMax, Boolean> initFunction) {
        m_initFunction = initFunction;
        m_sparkMax = sparkMax;
        m_sparkMaxEncoder = sparkMax.getEncoder();
        m_sparkMaxController = sparkMax.getPIDController();

        m_initFunction.apply(m_sparkMax);

        HealthMonitor.monitor(
            () -> sparkmaxMonitorFunction(m_sparkMax),
            () -> initFunction.apply(m_sparkMax));
    }

    private CANSparkMax m_sparkMax;
    private CANEncoder m_sparkMaxEncoder;
    private CANPIDController m_sparkMaxController;
    private Function<CANSparkMax, Boolean> m_initFunction;

    private static final int VELOCITY_GAIN_SLOT = 0;
    private static final int SMART_MOTION_GAIN_SLOT = 1;

	public Controller velocityController(PIDGains gains) {
        m_sparkMaxController.setP(gains.P, VELOCITY_GAIN_SLOT);
        m_sparkMaxController.setI(gains.I, VELOCITY_GAIN_SLOT);
        m_sparkMaxController.setD(gains.D, VELOCITY_GAIN_SLOT);
        return new ControllerSupplier(
            (ref, ff) -> {
                m_sparkMaxController.setReference(ref, ControlType.kVelocity, VELOCITY_GAIN_SLOT, ff);
            }
        );
	}

	public Encoder builtinEncoder() {
        return new EncoderSupplier(
            () -> m_sparkMaxEncoder.getVelocity(),
            () -> m_sparkMaxEncoder.getPosition(),
            (pos) -> m_sparkMaxEncoder.setPosition(pos)
        );
	}

	public Controller profiledController(PIDGains gains, Constraints constraints) {
        m_sparkMaxController.setP(gains.P, SMART_MOTION_GAIN_SLOT);
        m_sparkMaxController.setI(gains.I, SMART_MOTION_GAIN_SLOT);
        m_sparkMaxController.setD(gains.D, SMART_MOTION_GAIN_SLOT);
        m_sparkMaxController.setSmartMotionMaxAccel(constraints.maxAcceleration, SMART_MOTION_GAIN_SLOT);
        m_sparkMaxController.setSmartMotionMaxVelocity(constraints.maxVelocity, SMART_MOTION_GAIN_SLOT);
        return new ControllerSupplier(
            (ref, ff) -> {
                m_sparkMaxController.setReference(ref, ControlType.kSmartMotion, SMART_MOTION_GAIN_SLOT, ff);
            }
        );
    }
    
    public CANSparkMax getCANSparkMax() {
        return m_sparkMax;
    }
}
