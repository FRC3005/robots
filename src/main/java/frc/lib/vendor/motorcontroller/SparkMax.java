package frc.lib.vendor.motorcontroller;

import java.util.function.Function;

import frc.lib.controller.PIDGains;
import frc.lib.controller.Controller;
import frc.lib.controller.ControllerSupplier;
import frc.lib.electromechanical.Encoder;
import frc.lib.electromechanical.EncoderSupplier;
import frc.lib.electromechanical.MotorController;
import frc.lib.util.HealthMonitor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import frc.lib.util.AccessOrderHashSet;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.util.Constraints;

public class SparkMax implements MotorController {

    private CANSparkMax m_sparkMax;
    private RelativeEncoder m_sparkMaxEncoder;
    private SparkMaxPIDController m_sparkMaxController;
    private AccessOrderHashSet<Function<CANSparkMax, Boolean>> m_mutatorChain;
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

    /**
     * Create a motor controller from a CANSparkMax object. 
     * 
     * @param sparkMax A CANSparkMax object. Run any initialization that you want
     * excluded from the built in health monitor functions
     * 
     * @param initFunction A function which takes a CANSparkMax and returns a Boolean.
     * This function is used to initialize the CANSparkMax device, and is called in
     * one of two places. 1) It is called in this constructor, and 2) it is called
     * in the case of a health monitor timeout (i.e. the controller has reset)
     * 
     */
    public SparkMax(CANSparkMax sparkMax, Function<CANSparkMax, Boolean> initFunction) {
        m_mutatorChain = new AccessOrderHashSet<>();
        m_mutatorChain.add(initFunction);
        m_sparkMax = sparkMax;
        m_sparkMaxEncoder = sparkMax.getEncoder();
        m_sparkMaxController = sparkMax.getPIDController();

        initFunction.apply(m_sparkMax);

        HealthMonitor.monitor(
            () -> sparkmaxMonitorFunction(m_sparkMax),
            () -> initFunction.apply(m_sparkMax));
    }

    // Cache gains so 'get' commands don't go out on CAN 
    // 4 gain slots in spark max
    private PIDGains m_gainsCached[] = new PIDGains[4];
    private Constraints m_constraintsCached = new Constraints(0.0, 0.0);

    private static final int VELOCITY_GAIN_SLOT = 0;
    private static final int SMART_MOTION_GAIN_SLOT = 1;

    /**
     * Initialize a sendable for tuning the velocity controller PID constants
     * at run time
     * @param builder Builder passed in during initSendable
     */
    public void controllerInitSendable(SendableBuilder builder, int slot) {
        builder.addDoubleProperty("p",
            () -> m_gainsCached[slot].P,
            (val) -> {
                m_gainsCached[slot].P = val;
                m_sparkMaxController.setP(val, slot);
            });
        builder.addDoubleProperty("i",
            () -> m_gainsCached[slot].I,
            (val) -> {
                m_gainsCached[slot].I = val;
                m_sparkMaxController.setI(val, slot);
            });
        builder.addDoubleProperty("d",
            () -> m_gainsCached[slot].D,
            (val) -> {
                m_gainsCached[slot].D = val;
                m_sparkMaxController.setD(val, slot);
            });

        if (slot != SMART_MOTION_GAIN_SLOT) {
            return;
        }

        builder.addDoubleProperty("max accel",
            () -> m_constraintsCached.maxAcceleration,
            (val) -> {
                m_constraintsCached.maxAcceleration = val;
                m_sparkMaxController.setSmartMotionMaxAccel(val, SMART_MOTION_GAIN_SLOT);
            });
            
        builder.addDoubleProperty("max velocity",
            () -> m_constraintsCached.maxVelocity,
            (val) -> {
                m_constraintsCached.maxVelocity = val;
                m_sparkMaxController.setSmartMotionMaxVelocity(val, SMART_MOTION_GAIN_SLOT);
            });
    }

    /**
     * Create a Controller based on the internal spark max velocity controller
     * 
     * @param gains PID gains, note that units will depend on how the device
     * is configured so be sure to check this!!
     * 
     * @return
     */
	public Controller velocityController(PIDGains gains) {
        m_gainsCached[VELOCITY_GAIN_SLOT] = gains;

        // Add this to mutate list so it is reset if controller resets
        mutate(sm -> {
            m_sparkMaxController.setP(gains.P, VELOCITY_GAIN_SLOT);
            m_sparkMaxController.setI(gains.I, VELOCITY_GAIN_SLOT);
            m_sparkMaxController.setD(gains.D, VELOCITY_GAIN_SLOT);
            return true;
        });

        return new ControllerSupplier(
            (ref, ff) ->
                m_sparkMaxController.setReference(ref, CANSparkMax.ControlType.kVelocity, VELOCITY_GAIN_SLOT, ff),
            builder -> controllerInitSendable(builder, VELOCITY_GAIN_SLOT)
        );
	}

    /**
     * Create an encoder object based on the Spark Max internal encoder
     * 
     * @return Encoder object
     */
	public Encoder builtinEncoder() {
        return new EncoderSupplier(
            () -> m_sparkMaxEncoder.getVelocity(),
            () -> m_sparkMaxEncoder.getPosition(),
            pos -> m_sparkMaxEncoder.setPosition(pos)
        );
	}

    /**
     * Create a profiled PID controller based on the spark max smart motion
     * 
     * @param gains
     * @param constraints
     * @return
     */
	public Controller profiledController(PIDGains gains, Constraints constraints) {
        m_constraintsCached = constraints;
        m_gainsCached[SMART_MOTION_GAIN_SLOT] = gains;

        // Add this to mutate list so it is reset if controller resets
        mutate((sm) -> {
            m_sparkMaxController.setP(gains.P, SMART_MOTION_GAIN_SLOT);
            m_sparkMaxController.setI(gains.I, SMART_MOTION_GAIN_SLOT);
            m_sparkMaxController.setD(gains.D, SMART_MOTION_GAIN_SLOT);
            m_sparkMaxController.setSmartMotionMaxAccel(constraints.maxAcceleration, SMART_MOTION_GAIN_SLOT);
            m_sparkMaxController.setSmartMotionMaxVelocity(constraints.maxVelocity, SMART_MOTION_GAIN_SLOT);
            return true;
        });
        
        return new ControllerSupplier(
            (ref, ff) -> 
                m_sparkMaxController.setReference(ref, CANSparkMax.ControlType.kSmartMotion, SMART_MOTION_GAIN_SLOT, ff),
            builder -> 
                controllerInitSendable(builder, SMART_MOTION_GAIN_SLOT)
        );
    }

    /**
     * Modify CANSparkMax object. Mutations using this
     * method are re-run in order in the case of a device
     * failure that is later recovered. The same function can
     * be called multiple times, and will simply be moved to
     * the end of the list each call.
     * 
     * Only adds the function to the list if it succeeds
     * 
     * @param fcn a function on the underlying CANSparkMax object
     * returning true on success. Typically used to change parameter
     * values. Function should run quickly and return.
     * 
     * @return result of mutate function
     */
    public boolean mutate(Function<CANSparkMax, Boolean> fcn) {
        Boolean result = fcn.apply(m_sparkMax);
        if (result != null && result) {
            m_mutatorChain.add(fcn);
        }
        return result;
    }

    /**
     * Get the raw CANSparkMax object, only use if you need to
     * access functions from the CANSparkMax API. If you want to
     * modify the object and have the new state persist through
     * power loss or error, call mutate()
     * 
     * @return CANSparkMax object
     */
    public CANSparkMax getCANSparkMax() {
        return m_sparkMax;
    }
}
