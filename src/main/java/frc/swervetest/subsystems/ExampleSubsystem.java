// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swervetest.subsystems;

import frc.lib.electromechanical.Gearbox;
import frc.lib.electromechanical.ServoMotor;
import frc.lib.vendor.motorcontroller.SparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    /*
    SparkMax sparkMax1 = new SparkMax(
      new CANSparkMax(1, MotorType.kBrushless),
      (CANSparkMax device) -> {
        device.restoreFactoryDefaults();
        return false;
    });

    ServoMotor motor = new ServoMotor(sparkMax1,
      sparkMax1.velocityController(),
      sparkMax1.positionController(),
      sparkMax1.encoder(),
      new Gearbox(10,1).withEfficiency(0.75));
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
