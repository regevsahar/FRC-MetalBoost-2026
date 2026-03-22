package frc.lib.util.FieldUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.math.AngleTransform;
import frc.lib.math.FieldMath;
import java.util.Optional;

public class FieldPoses {

  // TODO: update these positions and name and add more if needed
  public enum CenteringPositions {
    TRANCH_LEFT(0, new Pose2d(3.5, 7.436, Rotation2d.fromDegrees(0))),

    TRANCH_RIGHT(1, new Pose2d(3.5, 0.66, Rotation2d.fromDegrees(0))),

    CLIMBER_LEFT_BLUE(2, new Pose2d(1.63, 3.2, Rotation2d.fromDegrees(180))),

    CLIMBER_RIGHT_BLUE(3, new Pose2d(1.63, 4.27, Rotation2d.fromDegrees(180))),

    CLIMBER_CENTER_BLUE(4, new Pose2d(1.63, 3.7, Rotation2d.fromDegrees(180))),

    BUMPER_LEFT_BLUE(5, new Pose2d(3.5, 5.5, Rotation2d.fromDegrees(0))),

    BUMPER_RIGHT_BLUE(6, new Pose2d(3.5, 2.5, Rotation2d.fromDegrees(0)));

    private final int index;
    private final Pose2d pose;

    CenteringPositions(int index, Pose2d pose) {
      this.index = index;
      this.pose = pose;
    }

    public int getIndex() {
      return index;
    }

    public Pose2d getPose() {
      return pose;
    }

    public double getDistance(Pose2d pose) {
      return this.pose.getTranslation().getDistance(pose.getTranslation());
    }
  }

  public static final Translation2d HUB_CENTER_RED = new Translation2d(11.915, 4.035);
  public static final Translation2d HUB_CENTER_BLUE = new Translation2d(4.625, 4.035);

  public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE =
      DriverStation.Alliance.Blue;

  public static boolean isFieldConventionAlliance() {
    return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
  }

  public static final double LENGTH_METERS = 16.54;
  public static final double WIDTH_METERS = 8.07;

  public static boolean isOnBlueSide(Translation2d robotTranslation) {
    return robotTranslation.getX() < LENGTH_METERS / 2.0;
  }

  public static Translation2d getHubPosByAliiance() {
    Translation2d hubPosition = HUB_CENTER_BLUE;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      hubPosition = HUB_CENTER_RED;
    }
    return hubPosition;
  }

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

  public static Translation2d getClosestBumper(Translation2d robotTranslation) {
    Translation2d leftBamper =
        getAllianceRelative(CenteringPositions.BUMPER_LEFT_BLUE.getPose().getTranslation());
    Translation2d rightBamper =
        getAllianceRelative(CenteringPositions.BUMPER_RIGHT_BLUE.getPose().getTranslation());

    if (robotTranslation.getDistance(leftBamper) < robotTranslation.getDistance(rightBamper)) {
      return leftBamper;
    }
    return rightBamper;
  }
}
