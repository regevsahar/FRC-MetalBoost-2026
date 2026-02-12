// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class HeightSpeedReduction {
  private static HeightSpeedReduction instance;
  private double height;
  private double speed;

  // Elevator height range (expected normal range, but can go up to ~120)
  private double minHeight = 17.0; // Height where speed starts decreasing
  private double maxHeight = 179.0; // Height where speed reaches minimum

  // Define the speed range
  private double maxSpeed = 0.75; // Speed at minHeight
  private double minSpeed = 0.15; // Speed at maxHeight (or higher)

  private HeightSpeedReduction() {
    height = 0;
    speed = 0.75;
  }

  public static HeightSpeedReduction getInstance() {
    if (instance == null) {
      instance = new HeightSpeedReduction();
    }
    return instance;
  }

  public void setHeight(double height) {
    this.height = height;
  }

  public void calculateSpeed() {
    // Normalize height to a 0-1 range (0 = minHeight, 1 = maxHeight)
    double heightProgress = (height - minHeight) / (maxHeight - minHeight);

    // Calculate the total speed reduction range
    double speedRange = maxSpeed - minSpeed; // Max possible speed drop (0.40 -> 40%)

    // Determine how much speed should be reduced based on height
    double speedReduction = heightProgress * speedRange; // Scales from 0 to 0.40 (0% -> 40%)

    // Subtract the reduction from max speed to get the final speed
    this.speed =
        maxSpeed - speedReduction; // Speed smoothly decreases as height increases (75% -> 35%)
  }

  /*
      EXAMPLE:

      Elevator Height      heightProgress   speedReduction   Final Speed
      -------------------------------------------------------------------
      17 cm (low)          0.00             0.00             0.75 (full speed)
      50 cm (middle)       0.40             0.16             0.59
      100 cm (high)        1.00             0.40             0.35 (slowest)
      120 cm (too high)    1.24             0.50             0.25 (extra slow)
  */

  public double getSpeed() {
    calculateSpeed();
    return speed;
  }

  public DoubleSupplier getSpeedSupplier() {
    return this::getSpeed;
  }
}
