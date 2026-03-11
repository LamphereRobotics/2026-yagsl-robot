// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public final class IntakeConstants {
  public static final int extendoMotorId = 14;
  public static final int intakeMotorId = 15;

  public static final SparkMaxConfig extendoConfig = new SparkMaxConfig();
  public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

  static {
    extendoConfig.inverted(true);

    intakeConfig.inverted(true);
  }
}
