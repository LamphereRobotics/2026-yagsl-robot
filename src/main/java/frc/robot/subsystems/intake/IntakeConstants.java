// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class IntakeConstants {
  public static final int intakeMotorId = 15;
  public static final double intakeVoltage = 12.0;

  public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

  static {
    intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20);
  }
}
