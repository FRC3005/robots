// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import frc.lib.electromechanical.ServoMotor;
import frc.lib.util.SendableHelper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
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
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getVelocity(), new Rotation2d(m_turningMotor.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningMotor.getPosition()));

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.setVelocity(state.speedMetersPerSecond);
    m_turningMotor.setPosition(state.angle.getRadians());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    SendableHelper.addChild(builder, this, m_driveMotor, "DriveMotor");
    SendableHelper.addChild(builder, this, m_turningMotor, "TurningMotor");
  }

  public void resetEncoders() {
    m_driveMotor.resetEncoder();
    m_turningMotor.resetEncoder();
  }
}

