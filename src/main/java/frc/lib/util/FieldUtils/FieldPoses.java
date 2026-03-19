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
    A_LEFT(0, true, new Pose2d(3.71123, 4.19654, Rotation2d.fromDegrees(0))),
    A_RIGHT(1, false, new Pose2d(3.71008, 3.86792, Rotation2d.fromDegrees(0))),

    B_LEFT(2, true, new Pose2d(3.95799, 3.44052, Rotation2d.fromDegrees(60))),
    B_RIGHT(3, false, new Pose2d(4.24201, 3.27522, Rotation2d.fromDegrees(60))),
    C_LEFT(4, true, new Pose2d(5.02126, 3.43953, Rotation2d.fromDegrees(120))),
    C_RIGHT(5, false, new Pose2d(4.73610, 3.27621, Rotation2d.fromDegrees(120))),

    D_LEFT(6, true, new Pose2d(5.26859, 4.19654, Rotation2d.fromDegrees(180))),
    D_RIGHT(7, false, new Pose2d(5.26745, 3.86792, Rotation2d.fromDegrees(180))),

    E_LEFT(8, true, new Pose2d(4.73667, 4.78924, Rotation2d.fromDegrees(-120))),
    E_RIGHT(9, false, new Pose2d(5.02069, 4.62394, Rotation2d.fromDegrees(-120))),

    F_LEFT(10, true, new Pose2d(4.24258, 4.78825, Rotation2d.fromDegrees(-60))),
    F_RIGHT(11, false, new Pose2d(3.93194, 4.62493, Rotation2d.fromDegrees(-60)));

    private final int index;
    private final boolean isLeft;
    private final Pose2d pose;

    CenteringPositions(int index, boolean isBlue, Pose2d pose) {
      this.index = index;
      this.isLeft = isBlue;
      this.pose = pose;
    }

    public int getIndex() {
      return index;
    }

    public boolean isLeft() {
      return isLeft;
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
}
