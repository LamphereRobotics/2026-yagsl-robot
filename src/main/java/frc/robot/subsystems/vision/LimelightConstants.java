package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public final class LimelightConstants {
  public static final String limelightNameAprilTag = "limelight-april";
  public static final String limelightNameShooter = "limelight-shooter";
  public static final Vector<N3> kMegaTag2VisionMeasurementStdDevs = VecBuilder.fill(.7, .7, 9999999);
}