// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;
import swervelib.SwerveDrive;

/** Add your docs here. */
public class Vision {
  public static double getShooterTX() {
    return LimelightHelpers.getTX(LimelightConstants.limelightNameShooter);
  }

  public static double getShooterTargetCount() {
    return LimelightHelpers.getTargetCount(LimelightConstants.limelightNameShooter);
  }

  public static void periodic(SwerveDrive swerveDrive) {
    updateLimelights(swerveDrive.getPose());

    if (shouldIgnoreVision(swerveDrive)) {
      return;
    }

    // addVisionMeasurements(swerveDrive);
  }

  private static void updateLimelights(Pose2d robotPose) {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(LimelightConstants.limelightNameAprilTag, 1); // Seed internal IMU
    } else {
      LimelightHelpers.SetIMUMode(LimelightConstants.limelightNameAprilTag, 4); // Use internal IMU + external IMU
    }

    Vision.setRobotOrientation(LimelightConstants.limelightNameAprilTag, robotPose);
    Vision.setRobotOrientation(LimelightConstants.limelightNameShooter, robotPose);
  }

  private static void setRobotOrientation(String limelightName, Pose2d robotPose) {
    LimelightHelpers.SetRobotOrientation(limelightName, robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
  }

  private static boolean shouldIgnoreVision(SwerveDrive swerveDrive) {
    // if our angular velocity is greater than 720 degrees per second, ignore vision
    // updates
    return Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) > Units.degreesToRadians(720);
  }

  private static void addVisionMeasurements(SwerveDrive swerveDrive) {
    final PoseEstimate[] estimates = {
        LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelightNameAprilTag),
        LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelightNameShooter)
    };

    for (var estimate : estimates) {
      if (estimate != null && estimate.tagCount > 0) {
        swerveDrive.setVisionMeasurementStdDevs(LimelightConstants.kMegaTag2VisionMeasurementStdDevs);
        swerveDrive.addVisionMeasurement(
            estimate.pose,
            estimate.timestampSeconds);
      }
    }
  }
}
