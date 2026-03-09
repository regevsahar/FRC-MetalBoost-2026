package frc.lib.util.Leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LedController {
  private Color ambiance = Color.kBlack;
  private final Color defaultColor = Color.kBlack;
  private static LedController instance;
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private volatile boolean flash = false;
  private boolean atSetPointActive = false;

  private LedController() {

    ledBuffer = new AddressableLEDBuffer(Constants.LedLength);

    led = new AddressableLED(Constants.LedId);
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public static LedController getInstance() {
    if (instance == null) {
      instance = new LedController();
    }
    return instance;
  }

  public void setData() {
    led.setData(ledBuffer);
  }

  public void startFlashing(Color color) {
    flash = true;
    new Thread(
            () -> {
              while (flash)
                try {
                  LEDPattern.solid(color).applyTo(ledBuffer);
                  setData();
                  Thread.sleep(100); // Flash ON

                  LEDPattern.solid(ambiance).applyTo(ledBuffer);
                  setData();
                  Thread.sleep(50);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
            })
        .start();
  }

  public void stopFlashing() {
    flash = false;
  }

  private volatile boolean gradientRunning = false;

  public void startGradient(Color... colors) {
    gradientRunning = true;
    new Thread(
            () -> {
              while (gradientRunning) {
                try {
                  LEDPattern base =
                      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
                  LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(40));
                  pattern.applyTo(ledBuffer);
                  setData();
                  Thread.sleep(20);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
              }
              returnToAmbiance();
            })
        .start();
  }

    public void startGradient(int speed, Color... colors) {
    gradientRunning = true;
    new Thread(
            () -> {
              while (gradientRunning) {
                try {
                  LEDPattern base =
                      LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colors);
                  LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(speed));
                  pattern.applyTo(ledBuffer);
                  setData();
                  Thread.sleep(20);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                }
              }
              returnToAmbiance();
            })
        .start();
  }

  public void stopGradient() {
    gradientRunning = false;
  }

  public void flash(Color color) {
    flash(color, 4, 100);
  }

  public void flash(Color color, int times, int waitTime) {
    new Thread(
            () -> {
              try {
                for (int i = 0; i < times; i++) {
                  LEDPattern.solid(color).applyTo(ledBuffer);
                  setData();
                  Thread.sleep(waitTime);

                  LEDPattern.solid(ambiance).applyTo(ledBuffer);
                  setData();
                  Thread.sleep(waitTime);
                }
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();
  }

  public void flash(Color color, int waitTime, boolean setPointCalled) {
    new Thread(
            () -> {
              try {
                LEDPattern.solid(color).applyTo(ledBuffer);
                setData();
                Thread.sleep(waitTime);

                LEDPattern.solid(ambiance).applyTo(ledBuffer);
                setData();
                if (setPointCalled) atSetPointActive = false;
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
              }
            })
        .start();
  }

  public void changeAmbiance(Color color) {
    ambiance = color;
    LEDPattern.solid(color).applyTo(ledBuffer);
    setData();
  }

  public void returnToAmbiance() {
    LEDPattern.solid(ambiance).applyTo(ledBuffer);
    setData();
  }

  public void atSetPoint() {
    if (atSetPointActive) return;
    atSetPointActive = true;
    flash(Color.kGreen, 1500, true);
  }

  public void centered() {
    flash(Color.kPurple);
  }

  public void error() {
    flash(Color.kDarkRed, 2, 100);
  }

  public void defaultAnimation() {
    defaultAnimation(40);
  }

  public void defaultAnimation(int speed) {
    LEDPattern base =
        LEDPattern.gradient(
            LEDPattern.GradientType.kContinuous,
            Color.kDarkBlue,
            Color.kMagenta,
            Color.kDarkBlue,
            Color.kDarkMagenta);

    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(speed));
    pattern.applyTo(ledBuffer);
    setData();
  }

  public void defaultColor() {
    LEDPattern.solid(defaultColor).applyTo(ledBuffer);
    ambiance = defaultColor;
    setData();
  }

  public void redAlliance() {
    LEDPattern.solid(Color.kRed).applyTo(ledBuffer);
    ambiance = Color.kRed;
    setData();
  }

  public void blueAlliance() {
    LEDPattern.solid(Color.kBlue).applyTo(ledBuffer);
    ambiance = Color.kBlue;
    setData();
  }
}
