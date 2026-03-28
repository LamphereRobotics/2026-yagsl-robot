// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.vision.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class Vision {
  public static void periodic(Pose2d robotPose) {
    if (DriverStation.isDisabled()) {
      LimelightHelpers.SetIMUMode(LimelightConstants.limelightNameAprilTag, 1); // Seed internal IMU
    } else {
      LimelightHelpers.SetIMUMode(LimelightConstants.limelightNameAprilTag, 4); // Use internal IMU + external IMU
    }

    Vision.setRobotOrientation(LimelightConstants.limelightNameAprilTag, robotPose);
    Vision.setRobotOrientation(LimelightConstants.limelightNameShooter, robotPose);
  }

  public static PoseEstimate[] useMegaTag2VisionEstimate() {
    final PoseEstimate[] mt2Estimates = {
        LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelightNameAprilTag),
        LimelightHelpers
            .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.limelightNameShooter)

    };

    return mt2Estimates;
  }

  public static double getShooterTX() {
    return LimelightHelpers.getTX(LimelightConstants.limelightNameShooter);
  }

  public static double getShooterTargetCount() {
    return LimelightHelpers.getTargetCount(LimelightConstants.limelightNameShooter);
  }

  private static void setRobotOrientation(String limelightName, Pose2d robotPose) {
    LimelightHelpers.SetRobotOrientation(limelightName, robotPose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
  }
}
