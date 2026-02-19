package frc.lib.util;

import frc.robot.subsystems.Shooter.ShooterConstants;

public class FlyWheelHoodIntegrationTable {

  public static double getRPMFromAngle(double angleDeg) {
    return ShooterConstants.ANGLE_TO_RPM_MAP.get(angleDeg);
  }
}
