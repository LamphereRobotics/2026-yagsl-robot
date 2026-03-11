// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class HopperConstants {
  public static final int conveyorMotorId = 13;
  public static final int indexerMotorId = 9;

  public static final SparkMaxConfig conveyorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig indexerConfig = new SparkMaxConfig();

  static {
    conveyorConfig.idleMode(IdleMode.kCoast);
    indexerConfig.inverted(true).idleMode(IdleMode.kBrake);
  }
}
