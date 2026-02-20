package frc.lib.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class FlightTimeTable {

  private static final InterpolatingDoubleTreeMap DISTANCE_TO_FLIGHT_TIME_MAP = new InterpolatingDoubleTreeMap();

  static {
    // TODO: Replace with real measurements
    // Distance (meters) -> Time (seconds)
    DISTANCE_TO_FLIGHT_TIME_MAP.put(0.0, 0.0);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(1.0, 0.2);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(2.0, 0.35);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(3.0, 0.5);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(4.0, 0.65);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(5.0, 0.8);
    DISTANCE_TO_FLIGHT_TIME_MAP.put(6.0, 1.0);
  }

  public static double get(double distanceMeters) {
    return DISTANCE_TO_FLIGHT_TIME_MAP.get(distanceMeters);
  }
}
