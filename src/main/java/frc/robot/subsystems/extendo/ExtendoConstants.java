// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extendo;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public final class ExtendoConstants {
  public static final int extendoMotorId = 14;
  public static final int limitSwitchPort = 9;

  public static final SparkMaxConfig extendoConfig = new SparkMaxConfig();

  static {
    extendoConfig.inverted(true).idleMode(IdleMode.kBrake);
  }
}
