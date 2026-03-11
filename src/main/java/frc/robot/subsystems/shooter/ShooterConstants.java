// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public final class ShooterConstants {
  public static final int kickerMotorId = 12;
  public static final int shooterLeaderMotorId = 10;
  public static final int shooterFollowerMotorId = 11;

  public static final SparkMaxConfig kickerConfig = new SparkMaxConfig();
  public static final SparkMaxConfig shooterLeaderConfig = new SparkMaxConfig();
  public static final SparkMaxConfig shooterFollowerConfig = new SparkMaxConfig();

  static {
    shooterLeaderConfig.inverted(true);

    shooterLeaderConfig.follow(shooterLeaderMotorId, true);
  }
}
