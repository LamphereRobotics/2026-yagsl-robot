// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final SparkMax intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(getName() + "/intake/voltage",
        intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
  }

  public Command inCommand() {
    return run(this::in);
  }

  public void in() {
    intakeMotor.setVoltage(12);
  }

  public Command outCommand() {
    return run(this::in);
  }

  public void out() {
    intakeMotor.setVoltage(-12);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    intakeMotor.stopMotor();
  }
}
