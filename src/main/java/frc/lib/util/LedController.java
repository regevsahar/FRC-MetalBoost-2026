package frc.lib.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.*;

public class LedController {
    private Color ambiance = Color.kBlack;
    private final Color defaultColor = Color.kBlack;
    private static LedController instance;
    private AddressableLED[] leds = new AddressableLED[10];
    private AddressableLEDBuffer ledBuffer;
    private AddressableLEDBufferView leftLeds;
    private AddressableLEDBufferView rightLeds;
    private volatile boolean flash = false;
    private boolean atSetPointActive = false;

    private LedController() {

        ledBuffer = new AddressableLEDBuffer(Constants.LedLength);

        int mid = (ledBuffer.getLength() - 1) / 2;
        leftLeds = ledBuffer.createView(0, mid);
        rightLeds = ledBuffer.createView(mid + 1, ledBuffer.getLength() - 1);

        for (int port = 0; port < 10; port++) {
            leds[port] = new AddressableLED(port);
            leds[port].setLength(ledBuffer.getLength());
            leds[port].setData(ledBuffer);
            leds[port].start();
        }
    }

    public static LedController getInstance() {
        if (instance == null) {
            instance = new LedController();
        }
        return instance;
    }

    public void setData() {
        for (AddressableLED l : leds) {
            l.setData(ledBuffer);
        }
    }

    public void startFlashing(Color color) {
        flash = true;
        new Thread(() -> {
            while(flash)
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
        }).start();
    }

    public void stopFlashing() {
        flash = false;
    }

    public void flash(Color color) { flash(color, 4, 100); }

    public void flash(Color color, int times, int waitTime) {
        new Thread(() -> {
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
        }).start();
    }

    public void flash(Color color, int waitTime, boolean setPointCalled) {
        new Thread(() -> {
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
        }).start();
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

    public void defaultAnimation() { defaultAnimation(40); }

    public void defaultAnimation(int speed) {
        LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,
                Color.kLightSkyBlue, Color.kAqua, Color.kDarkBlue, Color.kAqua);

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