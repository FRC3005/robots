// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervetest.custom;

import com.lib.controller.PIDGains;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Drivetrain {
        public static final int kFrontLeftDriveMotorPort = 3;
        public static final int kRearLeftDriveMotorPort = 7;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kRearRightDriveMotorPort = 5;
    
        public static final int kFrontLeftTurningMotorPort = 4;
        public static final int kRearLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kRearRightTurningMotorPort = 6;

        // Drive motor current limit in Amps
		public static final int kDriveMotorCurrentLimit = 50;
		public static final int kTurningMotorCurrentLimit = 50;
        public static final PIDGains kTurningMotorPIDGains = new PIDGains(0.0, 0.0, 0.0);

        // Max Velocity in m/s
        public static final double kTurningMotorMaxVelocity = 5.0;
        
        // Max acceleration in m/s^2
        public static final double kTurningMotorMaxAccel = 5.0;
		public static final double kMaxDriveSpeed = 5.0;
		public static PIDGains kDriveMotorPIDGains = new PIDGains(1.0, 0.0, 0.0);
        public static double kDriveGearRatio;
       
        public static final double kTrackWidth = 0.4191;
        public static final double kWheelBase = 0.4191;
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    }
}
