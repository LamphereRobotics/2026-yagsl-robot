// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkMax kickerMotor = new SparkMax(kickerMotorId, MotorType.kBrushless);
  private final SparkMax shooterLeaderMotor = new SparkMax(shooterLeaderMotorId, MotorType.kBrushless);
  private final SparkMax shooterFollowerMotor = new SparkMax(shooterFollowerMotorId, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {
    kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterLeaderMotor.configure(shooterLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterFollowerMotor.configure(shooterFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(getName() + "/kicker/voltage",
        kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + "/leader/voltage",
        shooterLeaderMotor.getAppliedOutput() * shooterLeaderMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + "/follower/voltage",
        shooterFollowerMotor.getAppliedOutput() * shooterFollowerMotor.getBusVoltage());
  }

  public Command shootCommand() {
    return run(this::shoot);
  }

  public void shoot() {
    kickerMotor.setVoltage(3.5);
    shooterLeaderMotor.setVoltage(5.25);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    kickerMotor.stopMotor();
    shooterLeaderMotor.stopMotor();
  }
}
