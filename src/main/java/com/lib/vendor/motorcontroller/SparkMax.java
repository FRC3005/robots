package com.lib.vendor.motorcontroller;

import java.util.function.Function;

import com.lib.controller.PIDGains;
import com.lib.controller.Controller;
import com.lib.electromechanical.Encoder;
import com.lib.electromechanical.MotorController;
import com.lib.util.HealthMonitor;
import com.revrobotics.CANSparkMax;
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

        m_initFunction.apply(m_sparkMax);

        HealthMonitor.monitor(
            () -> sparkmaxMonitorFunction(m_sparkMax),
            () -> initFunction.apply(m_sparkMax));
    }

    private CANSparkMax m_sparkMax;
    private Function<CANSparkMax, Boolean> m_initFunction;

	public Controller velocityController(PIDGains kDriveMotorPIDGains) {
		return null;
	}

	public Encoder builtinEncoder() {
		return null;
	}

	public Controller profiledController(PIDGains kturningmotorpidgains, Constraints constraints) {
		return null;
	}
}
