package frc.lib.enums;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum Reef {
  A_LEFT(0, ReefSide.A, true, new Pose2d(3.71123, 4.19654, Rotation2d.fromDegrees(0))),
  A_RIGHT(1, ReefSide.A, false, new Pose2d(3.71008, 3.86792, Rotation2d.fromDegrees(0))),

  B_LEFT(2, ReefSide.B, true, new Pose2d(3.95799, 3.44052, Rotation2d.fromDegrees(60))),
  B_RIGHT(3, ReefSide.B, false, new Pose2d(4.24201, 3.27522, Rotation2d.fromDegrees(60))),

  C_LEFT(4, ReefSide.C, true, new Pose2d(5.02126, 3.43953, Rotation2d.fromDegrees(120))),
  C_RIGHT(5, ReefSide.C, false, new Pose2d(4.73610, 3.27621, Rotation2d.fromDegrees(120))),

  D_LEFT(6, ReefSide.D, true, new Pose2d(5.26859, 4.19654, Rotation2d.fromDegrees(180))),
  D_RIGHT(7, ReefSide.D, false, new Pose2d(5.26745, 3.86792, Rotation2d.fromDegrees(180))),

  E_LEFT(8, ReefSide.E, true, new Pose2d(4.73667, 4.78924, Rotation2d.fromDegrees(-120))),
  E_RIGHT(9, ReefSide.E, false, new Pose2d(5.02069, 4.62394, Rotation2d.fromDegrees(-120))),

  F_LEFT(10, ReefSide.F, true, new Pose2d(4.24258, 4.78825, Rotation2d.fromDegrees(-60))),
  F_RIGHT(11, ReefSide.F, false, new Pose2d(3.93194, 4.62493, Rotation2d.fromDegrees(-60)));

  private final int index;
  private final ReefSide reefSide;
  private final boolean isLeft;
  private final Pose2d pose;

  Reef(int index, ReefSide reefSide, boolean isLeft, Pose2d pose) {
    this.index = index;
    this.reefSide = reefSide;
    this.isLeft = isLeft;
    this.pose = pose;
  }

  public int getIndex() {
    return index;
  }

  public ReefSide getReefSide() {
    return reefSide;
  }

  public boolean isLeft() {
    return isLeft;
  }

  public Pose2d getPose() {
    return pose;
  }

  public static Reef getBranchByReefSideAndSide(ReefSide side, boolean left) {
    for (Reef b : values()) {
      if (b.reefSide == side && b.isLeft == left) return b;
    }
    return null;
  }

  public double getDistance(Pose2d pose) {
    return this.pose.getTranslation().getDistance(pose.getTranslation());
  }
}
