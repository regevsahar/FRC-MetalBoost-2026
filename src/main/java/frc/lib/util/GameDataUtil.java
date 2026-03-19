package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class GameDataUtil {

  /** Returns true when the hub is active based on match time and shift schedule. */
  public static boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }

    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in
    // teleop.
    if (gameData == null || gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        // If we have invalid game data, assume hub is active.
        return true;
    }

    // Shift 1 is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = false;
    switch (alliance.get()) {
      case Red:
        shift1Active = !redInactiveFirst;
        break;
      case Blue:
        shift1Active = redInactiveFirst;
        break;
    }

    return isHubActiveAtTime(matchTime, shift1Active);
  }

  /**
   * Returns true when the hub is currently inactive but will activate within 3 seconds. Use this to
   * give drivers an early warning rumble before the hub switches on.
   */
  public static boolean isHubAboutToActivate() {
    // If hub is already active, no need to warn.
    if (isHubActive()) {
      return false;
    }

    // Only relevant during teleop with known game data.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
      return false;
    }

    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        return false;
    }

    boolean shift1Active = false;
    switch (alliance.get()) {
      case Red:
        shift1Active = !redInactiveFirst;
        break;
      case Blue:
        shift1Active = redInactiveFirst;
        break;
    }

    // Check if the hub will be active 3 seconds from now (i.e. 3 fewer seconds
    // remaining).
    double futureMatchTime = DriverStation.getMatchTime() - 3.0;
    return isHubActiveAtTime(futureMatchTime, shift1Active);
  }

  // ---- Shift-change detection ----

  /** Tracks the previous shift number so we can detect transitions. */
  private static int previousShift = -1;

  /**
   * Returns true exactly once each time the game shifts to a new phase. Should be polled
   * periodically (e.g. from a Trigger supplier).
   */
  public static boolean didShiftJustChange() {
    if (!DriverStation.isTeleopEnabled()) {
      previousShift = -1; // reset when not in teleop
      return false;
    }

    int current = getCurrentShift();
    if (current != previousShift) {
      previousShift = current;
      return true;
    }
    return false;
  }

  /**
   * Returns the current shift number (0-5) based on match time remaining.
   *
   * <ul>
   *   <li>0 = Transition (>130 s)
   *   <li>1 = Shift 1 (105-130 s)
   *   <li>2 = Shift 2 (80-105 s)
   *   <li>3 = Shift 3 (55-80 s)
   *   <li>4 = Shift 4 (30-55 s)
   *   <li>5 = End game (≤30 s)
   * </ul>
   */
  private static int getCurrentShift() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130) return 0;
    if (matchTime > 105) return 1;
    if (matchTime > 80) return 2;
    if (matchTime > 55) return 3;
    if (matchTime > 30) return 4;
    return 5;
  }

  // ---- Dashboard publishing ----

  /**
   * Publishes match timer and game-state information to SmartDashboard for the Elastic dashboard.
   * Call this from robotPeriodic().
   */
  public static void publishGameState() {
    double matchTime = DriverStation.getMatchTime();
    int shift = getCurrentShift();

    // Format match time as M:SS
    int totalSeconds = Math.max(0, (int) Math.ceil(matchTime));
    String formattedTime = String.format("%d:%02d", totalSeconds / 60, totalSeconds % 60);

    SmartDashboard.putString("/Game/Match Time", formattedTime);
    SmartDashboard.putNumber("/Game/Match Time Seconds", matchTime);
    SmartDashboard.putString("/Game/Current Shift", getShiftLabel(shift));
    SmartDashboard.putNumber("/Game/Shift Number", shift);
    SmartDashboard.putBoolean("/Game/Hub Active", isHubActive());
    SmartDashboard.putString("/Game/Game Phase", getGamePhase());
  }

  /** Returns a human-readable label for the given shift number. */
  private static String getShiftLabel(int shift) {
    switch (shift) {
      case 0:
        return "Transition";
      case 1:
        return "Shift 1";
      case 2:
        return "Shift 2";
      case 3:
        return "Shift 3";
      case 4:
        return "Shift 4";
      case 5:
        return "End Game";
      default:
        return "Unknown";
    }
  }

  /** Returns the current game phase as a string. */
  private static String getGamePhase() {
    if (DriverStation.isAutonomousEnabled()) {
      return "Autonomous";
    } else if (DriverStation.isTeleopEnabled()) {
      double matchTime = DriverStation.getMatchTime();
      if (matchTime <= 30) {
        return "End Game";
      }
      return "Teleop";
    } else if (DriverStation.isTestEnabled()) {
      return "Test";
    } else {
      return "Disabled";
    }
  }

  /** Computes whether the hub is active at a given match time remaining, given the shift state. */
  private static boolean isHubActiveAtTime(double matchTime, boolean shift1Active) {
    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }
}
