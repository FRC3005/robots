// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.lib.swerve;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Sendable;

public class SwerveDrive extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_rearLeft;
  private final SwerveModule m_rearRight;

  // The gyro sensor
  private final Gyro m_gyro;

  private final SwerveDriveKinematics m_kinematics;
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry;

  private final double m_maxSpeed;

  /**
   * Create a Swerve Drive module
   * @param frontLeft Swerve Module 
   * @param frontRight Swerve Module
   * @param rearLeft Swerve Module
   * @param rearRight Swerve Module
   * @param kinematics Swerve drive kinematics
   * @param gyro used for odometry and field centric driving
   * @param maxSpeed of the wheels used to normalize wheel speeds
   */
  public SwerveDrive(SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule rearLeft,
      SwerveModule rearRight,
      SwerveDriveKinematics kinematics,
      Gyro gyro,
      double maxSpeed) {
    m_frontLeft = frontLeft;
    m_frontRight = frontRight;
    m_rearLeft = rearLeft;
    m_rearRight = rearRight;
    m_gyro = gyro;
    m_kinematics = kinematics;
    m_maxSpeed = maxSpeed;
    m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d());
    initTelemetry();
  }

  private class ChassisSpeedsSendable implements Sendable {

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("ChassisSpeeds");

      builder.addDoubleArrayProperty(
        ".chassisSpeeds",
        () -> {
          ChassisSpeeds speeds = m_kinematics.toChassisSpeeds(getModuleStates());
          return new double[] {
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond
          };
        },
        null);
    }

  }

  private final ChassisSpeedsSendable m_chassisSpeedSendable = new ChassisSpeedsSendable();

  private void initTelemetry() {
    m_frontLeft.addParent(this, "frontLeft");
    m_frontRight.addParent(this, "frontRight");
    m_rearLeft.addParent(this, "rearLeft");
    m_rearRight.addParent(this, "rearRight");
    this.addChild("chassisSpeeds", m_chassisSpeedSendable);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        m_frontLeft.getState(),
        m_rearLeft.getState(),
        m_frontRight.getState(),
        m_rearRight.getState());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(
        swerveModuleStates, m_maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(
        desiredStates, m_maxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    // the order of this array MUST match the array in the drive constants
    return new SwerveModuleState[]{
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState(),
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }
}
