// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkFlex kickerMotor = new SparkFlex(kickerMotorId, MotorType.kBrushless);
  private final SparkFlex shooterLeaderMotor = new SparkFlex(shooterLeaderMotorId, MotorType.kBrushless);
  private final SparkFlex shooterFollowerMotor = new SparkFlex(shooterFollowerMotorId, MotorType.kBrushless);

  private final RelativeEncoder kickerEncoder = kickerMotor.getEncoder();
  private final RelativeEncoder shooterLeaderEncoder = shooterLeaderMotor.getEncoder();
  private final RelativeEncoder shooterFollowerEncoder = shooterFollowerMotor.getEncoder();

  private double targetVelocity = 0.0;

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
    SmartDashboard.putNumber(getName() + "/kicker/current", kickerMotor.getOutputCurrent());
    SmartDashboard.putNumber(getName() + "/kicker/velocity", kickerEncoder.getVelocity());

    SmartDashboard.putNumber(getName() + "/leader/voltage",
        shooterLeaderMotor.getAppliedOutput() * shooterLeaderMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + "/leader/current", shooterLeaderMotor.getOutputCurrent());
    SmartDashboard.putNumber(getName() + "/leader/velocity", shooterLeaderEncoder.getVelocity());

    SmartDashboard.putNumber(getName() + "/follower/voltage",
        shooterFollowerMotor.getAppliedOutput() * shooterFollowerMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + "/follower/current", shooterFollowerMotor.getOutputCurrent());
    SmartDashboard.putNumber(getName() + "/follower/velocity", shooterFollowerEncoder.getVelocity());
  }

  public Command shootHubCommand(Supplier<Distance> distanceToHub) {
    return run(() -> {
      shootHub(distanceToHub.get());
    });
  }

  public void shootHub(Distance distanceToHub) {
    final double output = distanceToHub.in(Meters) * voltPerMeter;
    kick();
    shoot(output);
  }

  public Command shootBlindCommand() {
    return run(this::shootBlind);
  }

  public void shootBlind() {
    kick();
    shoot(blindVoltage);
  }

  private void shoot(double output) {
    targetVelocity = output * velocityPerVolt;
    shooterLeaderMotor.setVoltage(output);
  }

  private void kick() {
    kickerMotor.setVoltage(kickVoltage);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    targetVelocity = 0.0;
    kickerMotor.stopMotor();
    shooterLeaderMotor.stopMotor();
  }

  public boolean isReadyToShoot() {
    return targetVelocity > 100.0 && shooterLeaderEncoder.getVelocity() > targetVelocity;
  }
}
