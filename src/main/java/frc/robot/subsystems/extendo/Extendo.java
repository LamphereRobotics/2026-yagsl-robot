// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extendo;

import static frc.robot.subsystems.extendo.ExtendoConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extendo extends SubsystemBase {
  private final SparkMax extendoMotor = new SparkMax(extendoMotorId, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Extendo() {
    extendoMotor.configure(extendoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(getName() + "/extendo/voltage",
        extendoMotor.getAppliedOutput() * extendoMotor.getBusVoltage());
  }

  public Command moveCommand(DoubleSupplier input) {
    return run(() -> move(input.getAsDouble()));
  }

  public void move(double input) {
    extendoMotor.setVoltage(input * 1.5);
  }

  public Command retractCommand() {
    return run(this::retract);
  }

  public void retract() {
    extendoMotor.setVoltage(1.5);
  }

  public Command extendCommand() {
    return run(this::extend);
  }

  public void extend() {
    extendoMotor.setVoltage(-1.5);
  }

  public Command stopCommand() {
    return run(this::stop);
  }

  public void stop() {
    extendoMotor.stopMotor();
  }
}
