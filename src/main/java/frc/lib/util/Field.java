package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.math.AngleTransform;
import frc.lib.math.FieldMath;

public class Field {

  public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE =
      DriverStation.Alliance.Blue;

  public static boolean isFieldConventionAlliance() {
    return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
  }

  public static final double LENGTH_METERS = 17.548225;
  public static final double WIDTH_METERS = 8.0518;

  public static boolean isOnBlueSide(Translation2d robotTranslation) {
    return robotTranslation.getX() < Field.LENGTH_METERS / 2.0;
  }

  private static final Translation2d REEF_MIDDLE = new Translation2d(4.48934, 4.03225);

  public static final double REEF_SIDE_LENGTH_METERS = 0.96;

  private static final Pose2d[] REEF_SIDE_MIDDLES =
      new Pose2d[] {
        new Pose2d(3.65760, 4.03220, Rotation2d.fromDegrees(0)), // A
        new Pose2d(4.07349, 3.31191, Rotation2d.fromDegrees(60)), // B
        new Pose2d(4.90523, 3.31193, Rotation2d.fromDegrees(120)), // C
        new Pose2d(5.32107, 4.03225, Rotation2d.fromDegrees(180)), // D
        new Pose2d(4.90519, 4.75254, Rotation2d.fromDegrees(-120)), // E
        new Pose2d(4.07345, 4.75252, Rotation2d.fromDegrees(-60)) // F
      };

  private static final Translation2d[] CORAL_BRANCHES =
      new Translation2d[] {
        new Translation2d(3.71123, 4.19654), // A
        new Translation2d(3.71008, 3.86792), // B
        new Translation2d(3.95799, 3.44052), // C
        new Translation2d(4.24201, 3.27522), // D
        new Translation2d(4.73610, 3.27621), // E
        new Translation2d(5.02126, 3.43953), // F
        new Translation2d(5.26745, 3.86792), // G
        new Translation2d(5.26859, 4.19654), // H
        new Translation2d(5.02069, 4.62394), // I
        new Translation2d(4.73667, 4.78924), // J
        new Translation2d(4.24258, 4.78825), // K
        new Translation2d(3.93194, 4.62493) // L
      };

  private static final Pose2d PROCESSOR = new Pose2d(5.98744, 0.02749, Rotation2d.fromDegrees(90));

  private static final Rotation2d RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(54);
  private static final Rotation2d LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-54);

  private static final Pose2d[] CORAL_STATION_MIDDLES =
      new Pose2d[] {
        new Pose2d(0.84319, 0.65078, RIGHT_CORAL_STATION_ANGLE),
        new Pose2d(0.84319, 7.41395, LEFT_CORAL_STATION_ANGLE)
      };

  public static Pose2d getAllianceRelative(
      Pose2d pose, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
    return isFieldConventionAlliance()
        ? pose
        : FieldMath.mirror(pose, mirrorX, mirrorY, angleTransform);
  }

  public static Translation2d getAllianceRelative(
      Translation2d translation, boolean mirrorX, boolean mirrorY) {
    return isFieldConventionAlliance()
        ? translation
        : FieldMath.mirror(translation, mirrorX, mirrorY);
  }

  public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
    return new Pose2d(
        point.getX() - point.getRotation().getCos() * distantInMeters,
        point.getY() - point.getRotation().getSin() * distantInMeters,
        point.getRotation());
  }

  public static Pose2d getAllianceRelative(Pose2d pose2d) {
    return new Pose2d(
        getAllianceRelative(pose2d.getTranslation()), getAllianceRelative(pose2d.getRotation()));
  }

  public static Translation2d getAllianceRelative(Translation2d translation) {
    return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, true, true);
  }

  public static Rotation2d getAllianceRelative(Rotation2d rotation) {
    return isFieldConventionAlliance()
        ? rotation
        : FieldMath.transformAngle(rotation, AngleTransform.INVERT);
  }
}
