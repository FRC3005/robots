// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervetest.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.ejml.simple.SimpleMatrix;

import com.revrobotics.CANSparkMaxSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimSparkMax extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(35, MotorType.kBrushless);
  private final SparkMaxPIDController m_pid;
  private final CANSparkMaxSim m_simMotor;
  private FlywheelSim m_dynamics;
  private double m_simVelocity = 0.0;

  /** Creates a new ExampleSubsystem. */
  public SimSparkMax() {
    Matrix<N1, N1> noiseStdDev = new Matrix<>(new SimpleMatrix(1, 1));
    noiseStdDev.set(0, 0, 0.1);
    m_simMotor = new CANSparkMaxSim(m_motor);
    m_dynamics = new FlywheelSim(DCMotor.getNEO(1), 1, 0.001, noiseStdDev);

    m_pid = m_motor.getPIDController();
    m_pid.setP(0.0005);
    m_pid.setOutputRange(-1.0, 1.0);
    m_motor.getEncoder().setPositionConversionFactor(2.0);
    m_motor.getEncoder().setVelocityConversionFactor(0.1);
  }

  public void setOutput(double value) {
    //m_motor.set(value * 5600);
    //m_simMotor.setSetpoint(value * 5600);
    m_pid.setReference(3000, CANSparkMax.ControlType.kVelocity);
    m_simMotor.setControlType(CANSparkMax.ControlType.kVelocity);
    m_simMotor.setSetpoint(3000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_simMotor.iterate(m_simVelocity, 12.0, 0.02);
    m_dynamics.setInputVoltage(m_simMotor.getAppliedOutput() * m_simMotor.getBusVoltage());
    m_dynamics.update(0.02);
    m_simVelocity = m_dynamics.getAngularVelocityRPM();
    m_simMotor.setMotorCurrent(m_dynamics.getCurrentDrawAmps());
    SmartDashboard.putNumber("Simulated Velocity", m_simVelocity);
  }
}
