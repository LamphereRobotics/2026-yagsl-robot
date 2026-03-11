// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  private final SparkMax conveyorMotor = new SparkMax(conveyorMotorId, MotorType.kBrushless);
  private final SparkMax indexerMotor = new SparkMax(indexerMotorId, MotorType.kBrushless);

  /** Creates a new Hopper. */
  public Hopper() {
    conveyorMotor.configure(conveyorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(getName() + "/conveyor/voltage",
        conveyorMotor.getAppliedOutput() * conveyorMotor.getBusVoltage());
    SmartDashboard.putNumber(getName() + "/indexer/voltage",
        indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage());
  }

  public Command inCommand() {
    return run(this::in);
  }

  public void in() {
    conveyorMotor.setVoltage(6);
    indexerMotor.setVoltage(6);
  }

  public Command outCommand() {
    return run(this::out);
  }

  public void out() {
    conveyorMotor.setVoltage(-6);
    indexerMotor.setVoltage(-6);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    conveyorMotor.stopMotor();
    indexerMotor.stopMotor();
  }
}
