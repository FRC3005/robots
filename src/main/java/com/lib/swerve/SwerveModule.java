// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lib.swerve;

import com.lib.electromechanical.ServoMotor;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule implements Sendable {
  private final ServoMotor m_driveMotor;
  private final ServoMotor m_turningMotor;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      ServoMotor driveMotor,
      ServoMotor turningMotor) {

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getVelocity(), new Rotation2d(m_turningMotor.getPosition()));
  }

  public void addParent(Sendable parent, String name) {
    SendableRegistry.addLW(this, SendableRegistry.getSubsystem(parent), name);
    SendableRegistry.addLW(m_turningPIDController, SendableRegistry.getSubsystem(parent), name + "/turningPID");
    SendableRegistry.addLW(m_drivePIDController, SendableRegistry.getSubsystem(parent), name +  "/drivePID");
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.setVelocity(state.speedMetersPerSecond);
    m_turningMotor.setPosition(state.angle.getRadians());
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition( 0 );
    m_turningEncoder.setPosition( getAbsoluteAngle() );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");

    builder.addStringProperty(
      ".driveMotor",
      () -> String.valueOf(m_driveMotor.getDeviceId()),
      null);
    builder.addStringProperty(
      ".turningMotor",
      () -> String.valueOf(m_turningMotor.getDeviceId()),
      null);

    builder.addDoubleProperty(
      ".turningAngleDegress",
      () -> m_turningEncoder.getPosition() * 180 / Math.PI,
      null);

    builder.addDoubleProperty(
      ".turingAngleAbsolute",
      () -> getAbsoluteAngle(),
      null);

    builder.addDoubleProperty(
      ".turingAngleAbsoluteRaw",
      () -> (1.0 - m_turningAbsolute.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI,
      null);
  }
}

