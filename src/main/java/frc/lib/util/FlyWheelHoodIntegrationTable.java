package frc.lib.util;

import java.util.Map;
import java.util.TreeMap;

public class FlyWheelHoodIntegrationTable {

  private static final TreeMap<Double, Double> ANGLE_TO_RPM_MAP = new TreeMap<>();

  static {
    ANGLE_TO_RPM_MAP.put(38.0, 2800.0);
    ANGLE_TO_RPM_MAP.put(60.0, 3200.0);
    ANGLE_TO_RPM_MAP.put(70.0, 3500.0);
    ANGLE_TO_RPM_MAP.put(82.0, 3400.0);
  }

  public static double getRPMFromAngle(double angleDeg) {
    return interpolate(ANGLE_TO_RPM_MAP, angleDeg);
  }

  private static double interpolate(TreeMap<Double, Double> map, double x) {
    Map.Entry<Double, Double> lower = map.floorEntry(x);
    Map.Entry<Double, Double> upper = map.ceilingEntry(x);

    if (lower == null && upper == null) return 0.0;
    if (lower == null) return map.firstEntry().getValue();
    if (upper == null) return map.lastEntry().getValue();

    if (lower.getKey().equals(upper.getKey())) {
      return lower.getValue();
    }

    double x0 = lower.getKey();
    double y0 = lower.getValue();
    double x1 = upper.getKey();
    double y1 = upper.getValue();

    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}
