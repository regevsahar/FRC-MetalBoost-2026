package frc.lib.util;

import edu.wpi.first.hal.HALUtil;

public class TimeUtil {

	public static final double DEFAULT_CYCLE_TIME_SECONDS = 0.02;

	private static double lastCycleStartingTimeSeconds = 0;
	private static double currentCycleStartingTimeSeconds = 0;

	public static double getCurrentTimeSeconds() {
		return HALUtil.getFPGATime() / 1e6;
	}

	public static double getLatestCycleTimeSeconds() {
		return currentCycleStartingTimeSeconds - lastCycleStartingTimeSeconds;
	}

}
