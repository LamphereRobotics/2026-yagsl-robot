// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public final class ShooterConstants {
  public static final int kickerMotorId = 12;
  public static final int shooterLeaderMotorId = 10;
  public static final int shooterFollowerMotorId = 11;

  public static final SparkFlexConfig kickerConfig = new SparkFlexConfig();
  public static final SparkFlexConfig shooterLeaderConfig = new SparkFlexConfig();
  public static final SparkFlexConfig shooterFollowerConfig = new SparkFlexConfig();

  static {
    kickerConfig.idleMode(IdleMode.kCoast);

    shooterLeaderConfig.inverted(true).idleMode(IdleMode.kCoast);

    shooterFollowerConfig.follow(shooterLeaderMotorId, true).idleMode(IdleMode.kCoast);
  }
}
