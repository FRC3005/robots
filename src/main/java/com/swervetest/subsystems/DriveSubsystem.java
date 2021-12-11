package com.swervetest.subsystems;

import java.util.function.Function;

import com.lib.electromechanical.Gearbox;
import com.lib.electromechanical.ServoMotor;
import com.lib.controller.Controller;
import com.lib.swerve.SwerveDrive;
import com.lib.swerve.SwerveModule;
import com.lib.vendor.motorcontroller.SparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervetest.Constants;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class DriveSubsystem extends SwerveDrive {

    private static final Function<CANSparkMax, Boolean> DriveMotorInitializer = (CANSparkMax sparkMax) -> {
        sparkMax.restoreFactoryDefaults();
        CANEncoder enc = sparkMax.getEncoder();
    
        // Convert 'rotations' to 'meters'
        enc.setPositionConversionFactor(Constants.Drivetrain.kDriveEncoderPositionFactor);
    
        // Convert 'RPM' to 'meters per second'
        enc.setVelocityConversionFactor(Constants.Drivetrain.kDriveEncoderVelocityFactor);
    
        // Set inversion
        sparkMax.setInverted(Constants.Drivetrain.kDriveEncoderReversed);
    
        sparkMax.getPIDController().setOutputRange(-1, 1);
        sparkMax.setIdleMode(IdleMode.kCoast);
        sparkMax.setSmartCurrentLimit(Constants.Drivetrain.kDriveMotorCurrentLimit);
        sparkMax.burnFlash();
        return true;
    };

    private static final Function<CANSparkMax, Boolean> TurningMotorInitializer = (CANSparkMax sparkMax) -> {
        sparkMax.restoreFactoryDefaults();
        CANEncoder enc = sparkMax.getEncoder();
    
        // Convert 'rotations' to 'meters'
        enc.setPositionConversionFactor(Constants.Drivetrain.kTurningEncoderPositionFactor);
    
        // Convert 'RPM' to 'meters per second'
        enc.setVelocityConversionFactor(Constants.Drivetrain.kTurningEncoderVelocityFactor);
    
        // Set inversion
        sparkMax.setInverted(Constants.Drivetrain.kTurningEncoderReversed);
    
        sparkMax.getPIDController().setOutputRange(-1, 1);
        sparkMax.setIdleMode(IdleMode.kCoast);
        sparkMax.setSmartCurrentLimit(Constants.Drivetrain.kTurningMotorCurrentLimit);
        sparkMax.burnFlash();
        return true;
    };

    private static SwerveModule createSwerveModule(int driveMotorPort, int turningMotorPort) {
        SparkMax driveController = new SparkMax(
            new CANSparkMax(driveMotorPort, MotorType.kBrushless),
            DriveMotorInitializer
        );

        SparkMax turningController = new SparkMax(
            new CANSparkMax(turningMotorPort, MotorType.kBrushless),
            TurningMotorInitializer
        );

        ServoMotor driveMotor = new ServoMotor(driveController,
            driveController.velocityController(Constants.Drivetrain.kDriveMotorPIDGains),
            null,
            driveController.builtinEncoder(),
            new Gearbox(Constants.Drivetrain.kDriveGearRatio)
        );

        ServoMotor turningMotor = new ServoMotor(turningController,
            null,
            turningController.profiledController(
                Constants.Drivetrain.kTurningMotorPIDGains,
                new Constraints(
                    Constants.Drivetrain.kTurningMotorMaxVelocity,
                    Constants.Drivetrain.kTurningMotorMaxAccel)),
            turningController.builtinEncoder(),
            new Gearbox(10.0, 1.0)
        );

        return new SwerveModule(driveMotor, turningMotor);
    }

    static final SwerveModule frontLeft = createSwerveModule(Constants.Drivetrain.kFrontLeftDriveMotorPort, Constants.Drivetrain.kFrontLeftTurningMotorPort);
    static final SwerveModule frontRight = createSwerveModule(Constants.Drivetrain.kFrontRightDriveMotorPort, Constants.Drivetrain.kFrontRightTurningMotorPort);
    static final SwerveModule rearLeft = createSwerveModule(Constants.Drivetrain.kRearLeftDriveMotorPort, Constants.Drivetrain.kRearLeftTurningMotorPort);
    static final SwerveModule rearRight = createSwerveModule(Constants.Drivetrain.kRearRightDriveMotorPort, Constants.Drivetrain.kRearRightTurningMotorPort);

    public DriveSubsystem() {
        super(frontLeft, frontRight, rearLeft, rearRight, Constants.Drivetrain.kDriveKinematics, new ADXRS450_Gyro(), Constants.Drivetrain.kMaxDriveSpeed);
    }

}