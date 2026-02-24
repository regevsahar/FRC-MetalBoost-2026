package frc.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * missing.
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";
  private final String key;
  private double defaultValue;
  private double lastValue;
  private boolean hasDefault = false;

  /**
   * Create a new TunableNumber
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new TunableNumber with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableNumber(String dashboardKey, double defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    }
    return SmartDashboard.getNumber(key, defaultValue);
  }

  /**
   * Set the default value of the number. The default value can be updated.
   *
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (!hasDefault) {
      hasDefault = true;
      if (!SmartDashboard.containsKey(key)) {
        SmartDashboard.putNumber(key, defaultValue);
      }
    } else {
      if (!SmartDashboard.containsKey(key)) {
        SmartDashboard.putNumber(key, defaultValue);
      }
    }
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastValue) {
      lastValue = currentValue;
      return true;
    }
    return false;
  }
}
