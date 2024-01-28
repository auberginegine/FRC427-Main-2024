package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.leds.patterns.LEDPattern;

public class LEDStrip {
    //Creates buffer/pattern
    public AddressableLed ledBuffer;

    private LEDPattern pattern;

    //Sets pattern to whatever is input, then redirects to LEDStrip below
    public LEDStrip(AddressableLEDBuffer buffer, int port, int length, LEDPattern pattern) {
        this(buffer, port, length);
        this.pattern = pattern; 
    }
    //Crates Addressable:ED with buffer, start/end
    public LEDStrip(AddressableLEDBuffer buffer, int start, int end) {
        this.ledBuffer = new AddressableLed(buffer, start, end);
    }
    
    
    //Sets Pattern
    public void setPattern(LEDPattern pattern) {
        this.pattern = pattern;
    }
    //If there's a pattern, updates pattern with time
    public void update(double time) {
        if (pattern != null) pattern.update(ledBuffer, time);
    }

    
}