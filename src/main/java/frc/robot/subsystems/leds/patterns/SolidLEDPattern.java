package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;

public class SolidLEDPattern extends LEDPattern {

    private final Color color;

    public SolidLEDPattern(Color color) {
        this(color, 1); 
    }

    public SolidLEDPattern(Color color, double time) {
        super(time);
        this.color = color;
    }

    @Override
    protected void updateLEDs(AddressableLed buf, double time) {
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setLED(i, color);
        }
    }

}
