package com.revrobotics;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimEnum;
import edu.wpi.first.hal.SimInt;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;

import org.tinylog.Logger;

public class CANSparkMaxSim {
    private final SimDouble m_appliedOutput;
    private final SimDouble m_velocity;
    private final SimDouble m_position;
    private final SimDouble m_busVoltage;
    private final SimDouble m_motorCurrent;
    private final CANSparkMax m_sparkMax;
    private final DCMotor m_dcMotor;
    private ControlType m_controlMode = ControlType.kDutyCycle;
    private double m_setpoint = 0.0;
    private double m_arbFF = 0.0;
    private int m_pidSlot = 0;
    private int m_arbFFUnits = 0;
    private boolean m_forwardLimit = false;
    private boolean m_reverseLimit = false;

    // PID State
    double m_iState = 0.0;
    double m_prev_err = 0.0;
    double m_pidOutput = 0.0;

    /**
     * This *should* track to the API version. Since this file is
     * dependent on REV, warn if the version changes to go back
     * and verify 1) this still works and 2) that this file is still
     * needed at all.
     */
    private final int kAPIversionExpected = 17104900;

    /**
     * Create a simulated CAN Spark Max object. This class simulates
     * some of the internal behavior of the device.
     * 
     * @param sparkMax The CANSparkMax to simulate.
     * 
     * @param motor The motor connected to the SPARK MAX. If you are not
     * modeling the motor externally this can be used instead. If you have
     * a second motor controller as a follower, you can use a single
     * CANSparkMaxSim object on the leader, and specify multiple motors
     * using the DCMotor constructor.
     */
    public CANSparkMaxSim(CANSparkMax sparkMax, DCMotor motor) {

        SimDeviceSim sparkMaxSim = new SimDeviceSim("SPARK MAX" + " [" + sparkMax.getDeviceId() + "]");
        m_appliedOutput = sparkMaxSim.getDouble("Applied Output");
        m_position = sparkMaxSim.getDouble("Position");
        m_velocity = sparkMaxSim.getDouble("Velocity");
        m_busVoltage = sparkMaxSim.getDouble("Bus Voltage");
        m_motorCurrent = sparkMaxSim.getDouble("Motor Current");
        m_sparkMax = sparkMax;
        m_dcMotor = motor;

        int apiVersion = CANSparkMaxJNI.c_SparkMax_GetAPIVersion();
        if (apiVersion != kAPIversionExpected) {
            Logger.tag("CANSparkMaxSim")
                .warn("CAN Spark Max API version changed, verify that the sim setup still works correctly. Got {} expected {}",
                apiVersion, kAPIversionExpected);
        }
    }

    /**
     * Create a simulated CAN Spark Max object. This class simulates
     * some of the internal behavior of the device.
     * 
     * This constructor uses a single NEO as default, however if you
     * simulating the motor using a different sim mechanism it can
     * be ignored and setMotorCurrent() can be used instead.
     * 
     * @param sparkMax The CANSparkMax to simulate.
     */
    public CANSparkMaxSim(CANSparkMax sparkMax) {
        this(sparkMax, DCMotor.getNEO(1));
    }

    /**
     * Get the simulated applied output. This matches the value from
     * the CANSparkMax getAppliedOutput(). Multiply by vbus to get the
     * motor voltage.
     * 
     * @return applied output [-1, 1]
     */
    public double getAppliedOutput() {
        return m_appliedOutput.get();
    }

    /**
     * Set the simulated applied output. Use this only in place of iterate().
     * 
     * @param appliedOutput simulated applied output value [-1, 1]
     */
    public void setAppliedOutput(double appliedOutput) {
        m_appliedOutput.set(appliedOutput);
    }

    /**
     * Set the control type, call this after setReference or set()
     * 
     * This is a workaround while SimValue doesn't work.
     * 
     * @param controlType
     */
    public void setControlType(ControlType controlType) {
        m_controlMode = controlType;
    }

    /**
     * Set the setpoint command sent to the controller. Call this immediately after
     * the CANSparkMax.set() or setReference() from the actual cass.
     * This function is needed since the setpoint state from the SPARK MAX is
     * not accessible
     * 
     * @param setpoint The setpoint value sent to the controller
     * @param PIDSlot The 'PIDSlot' currently active
     * @param arbFF The arbitrary feedforward value
     * @param arbFFUnits The units for the arbitrary feedforward (0 = volts, 1 = %vbus)
     */
    public void setSetpoint(double setpoint, int PIDSlot, double arbFF, int arbFFUnits) {
        m_setpoint = setpoint;
        m_arbFF = arbFF;
        m_pidSlot = PIDSlot;
        m_arbFFUnits = arbFFUnits;
    }

    /**
     * Set the setpoint command sent to the controller. Call this immediately after
     * the CANSparkMax.set() or setReference() from the actual cass.
     * This function is needed since the setpoint state from the SPARK MAX is
     * not accessible
     * 
     * @param setpoint The setpoint value sent to the controller
     * @param PIDSlot The 'PIDSlot' currently active
     * @param arbFF The arbitrary feedforward value in volts
     */
    public void setSetpoint(double setpoint, int PIDSlot, double arbFF) {
        setSetpoint(setpoint, PIDSlot, arbFF, 0);
    }

    /**
     * Set the setpoint command sent to the controller. Call this immediately after
     * the CANSparkMax.set() or setReference() from the actual cass.
     * This function is needed since the setpoint state from the SPARK MAX is
     * not accessible
     * 
     * @param setpoint The setpoint value sent to the controller
     * @param PIDSlot The 'PIDSlot' currently active
     */
    public void setSetpoint(double setpoint, int PIDSlot) {
        setSetpoint(setpoint, PIDSlot, 0.0, 0);
    }

    /**
     * Set the setpoint command sent to the controller. Call this immediately after
     * the CANSparkMax.set() or setReference() from the actual cass.
     * This function is needed since the setpoint state from the SPARK MAX is
     * not accessible
     * 
     * @param setpoint The setpoint value sent to the controller
     */
    public void setSetpoint(double setpoint) {
        setSetpoint(setpoint, 0, 0.0, 0);
    }

    // Modified from https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
    private double runPID(double setpoint, double pv, int slot, double dt) {
        // TODO: factor in dt
        CANPIDController controller = m_sparkMax.getPIDController();
        double error = setpoint - pv;

        double p = error * controller.getP(slot);

        if(Math.abs(error) <= controller.getIZone(slot) || controller.getIZone(slot) == 0.0f) {
            m_iState = m_iState + (error * controller.getI(slot));
        } else {
            m_iState = 0;
        }

        double d = (error - m_prev_err);
        m_prev_err = error;
        d *= controller.getD(slot);

        double f = setpoint * controller.getFF(slot);

        double output = p + m_iState + d + f;
        m_pidOutput = Math.min(Math.max(output, controller.getOutputMin(slot)), controller.getOutputMax(slot));

        return m_pidOutput;
    }

    // return true if limited (i.e. output should be 0 in this direction)
    private boolean runLimitLogic(boolean forward) {
        if (forward) {
            if ((m_sparkMax.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get()) &&
                    CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(m_sparkMax.m_sparkMax, SoftLimitDirection.kForward.value)) {
                return true;
            }

            return (CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(m_sparkMax.m_sparkMax, LimitSwitch.kForward.value) && m_forwardLimit);
        } else {
            if ((m_sparkMax.getSoftLimit(SoftLimitDirection.kReverse) > m_position.get()) &&
                    CANSparkMaxJNI.c_SparkMax_IsSoftLimitEnabled(m_sparkMax.m_sparkMax, SoftLimitDirection.kReverse.value)) {
                return true;
            }

            return (CANSparkMaxJNI.c_SparkMax_IsLimitEnabled(m_sparkMax.m_sparkMax, LimitSwitch.kReverse.value) && m_reverseLimit);
        }
    }

    /**
     * Run internal calculations and set internal state including
     * 
     * - Velocity (set by velocity, not calculated in this method)
     * - Position
     * - Bus Voltage (set by vbus, not calculated in this method)
     * - Current
     * - Applied Output
     * 
     * @param velocity The externally calculated velocity in units after
     * conversion. For example, if the velocity factor is 1, use RPM. If the
     * velocity factor is (1 / 60) use RPS.
     *
     * @param vbus Bus voltage in volts
     *
     * @param dt Simulation time step in seconds
     */
    public void iterate(double velocity, double vbus, double dt) {
        // First set the states that are given
        m_velocity.set(velocity);

        // TODO: This doesn't work with the 2021 SPARK MAX API
        double positionFactor = CANSparkMaxJNI.c_SparkMax_GetPositionConversionFactor(m_sparkMax.m_sparkMax);
        double velocityFactor = CANSparkMaxJNI.c_SparkMax_GetVelocityConversionFactor(m_sparkMax.m_sparkMax);

        // These come back as 0, maybe an API issue?
        if (positionFactor == 0.0) {
            positionFactor = 1.0;
        }
        if (velocityFactor == 0.0) {
            velocityFactor = 1.0;
        }

        double velocityRPM = velocity / velocityFactor;
        m_position.set(
            m_position.get() + (velocityRPM * dt) / positionFactor
        );
        m_busVoltage.set(vbus);

        // Calcuate the applied output
        double appliedOutput = 0.0;
        switch (m_controlMode) {
            // Duty Cycle
            case kDutyCycle:
            appliedOutput = m_setpoint;
            break;

            // Velocity
            case kVelocity:
            appliedOutput = runPID(m_setpoint, velocity, m_pidSlot, dt);
            break;

            // Voltage
            case kVoltage:
            appliedOutput = m_setpoint / vbus;
            break;

            // Position
            case kPosition:
            appliedOutput = runPID(m_setpoint, m_position.get(), m_pidSlot, dt);
            break;
            // Smart Motion
            case kSmartMotion:
            // TODO... This control mechansim is not documented
            break;

            // Current
            case kCurrent:
            appliedOutput = runPID(m_setpoint, m_motorCurrent.get(), m_pidSlot, dt);
            break;

            // Smart Velocity
            case kSmartVelocity:
            // TODO... This control mechansim is not documented
            break;

            default:
            Logger.tag("CANSparkMaxSim").error("Invalid control mode: {}", m_controlMode.value);
        }

        // ArbFF
        if (m_arbFFUnits == 0) {
            // Voltage
            appliedOutput += m_arbFF / vbus;
        } else {
            // Duty Cycle
            appliedOutput += m_arbFF;
        }

        // Limit to [-1, 1] or limit switch value
        double maxOutput = runLimitLogic(true) ? 0 : 1;
        double minOutput = runLimitLogic(false) ? 0 : -1;
        appliedOutput = Math.min(Math.max(appliedOutput, minOutput), maxOutput);

        // TODO: Voltage Comp

        // TODO: Selected Sensor?

        // TODO: Faults

        // And finally, set remaining states
        m_appliedOutput.set(appliedOutput);
        m_motorCurrent.set(m_dcMotor.getCurrent(
            velocityRPM, 
            appliedOutput * vbus
        ));

        // TODO: Current Limits
    }

    /**
     * Get the simulation velocity. This should be equivilant to calling
     * CANEncoder().getVelocity()
     * 
     * @return Velocity of the SPARK MAX accounting for conversion factor
     */
    public double getVelocity() {
        return m_velocity.get();
    }

    /**
     * Set the simulation velocity. This method expects units
     * after the conversion factor (your program's native units).
     * 
     * Only use this method if not calling iterate()
     * 
     * @param velocity simulation velocity
     */
    public void setVelocity(double velocity) {
        m_velocity.set(velocity);
    }

    /**
     * Get the simulation position. This should be equivilant to calling
     * CANEncoder().getPosition()
     * 
     * @return Velocity of the SPARK MAX
     */
    public double getPosition() {
        return m_position.get();
    }

    /**
     * Set the simulated position. This is equivilant to calling
     * CANEncoder().setPosition(), in fact you probably are using that
     * unless you have a good reason to set the sim value separately,
     * or are running simulation without using iterate()
     * 
     * @param position simulated position in your programs units (after conversion)
     */
    public void setPosition(double position) {
        m_position.set(position);
    }

    /**
     * Get the simulated bus voltage
     * 
     * @return simulated bus voltage in volts
     */
    public double getBusVoltage() {
        return m_busVoltage.get();
    }

    /**
     * Set the simulated bus voltage. Use this if you are not using the
     * iterate() method.
     * 
     * @param voltage bus voltage in volts
     */
    public void setBusVoltage(double voltage) {
        m_busVoltage.set(voltage);
    }

    /**
     * Get the simulated motor current in amps. This is equivilant to running
     * sparkmax.getOutputCurrent()
     * 
     * @return motor current in amps
     */
    public double getMotorCurrent() {
        return m_motorCurrent.get();
    }

    /**
     * Set the simulated motor current. The iterate() method also sets this value.
     * If you are using an external method to calculate the current, but still want
     * to use the iterate() method, call this function *after* iterate()
     * 
     * @param current current in amps
     */
    public void setMotorCurrent(double current) {
        m_motorCurrent.set(current);
    }

    /**
     * Set the state of the forward limit switch. Set true to indicate that
     * the forward limit switch is set.
     * 
     * This method does not have any knowlege of polarity. So if you set the
     * Spark Max limit switch as 'normally open' and set tripped = true, then
     * the limit is concidered closed (tripped). If you set the Spark Max limit
     * switch as 'normally closed' as set tripped = true, then the limit switch
     * is considered open (tripped).
     * 
     * @param tripped set true to trip the forward limit
     */
    public void setForwardLimitSwitch(boolean tripped) {
        m_forwardLimit = tripped;
    }

    /**
     * Get the simulated forward limit switch state
     * 
     * @return true if tripped. Does not look at whether or not enabled
     */
    public boolean getForwardLimitSwitch() {
        return m_forwardLimit;
    }

    /**
     * Set the state of the reverse limit switch. Set true to indicate that
     * the reverse limit switch is set.
     * 
     * This method does not have any knowlege of polarity. So if you set the
     * Spark Max limit switch as 'normally open' and set tripped = true, then
     * the limit is concidered closed (tripped). If you set the Spark Max limit
     * switch as 'normally closed' as set tripped = true, then the limit switch
     * is considered open (tripped).
     * 
     * @param tripped set true to trip the reverse limit
     */
    public void setReverseLimitSwitch(boolean tripped) {
        m_reverseLimit = tripped;
    }

    /**
     * Get the simulated reverse limit switch state
     * 
     * @return true if tripped. Does not look at whether or not enabled
     */
    public boolean getReverseLimitSwitch() {
        return m_reverseLimit;
    }
}
