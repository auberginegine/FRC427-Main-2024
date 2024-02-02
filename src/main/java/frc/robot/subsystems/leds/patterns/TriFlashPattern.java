package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;
import edu.wpi.first.wpilibj.Timer;


public class TriFlashPattern extends LEDPattern{
    Color color1;
    Color color2;
    Color color3;
    private Timer timer = new Timer();
    
    

    public TriFlashPattern(Color color1, Color color2, Color color3) {
        this(color1, color2, color3, 1); 
        
    }
    
    public TriFlashPattern(Color color1, Color color2,Color color3, double time) {
        super(time);
        this.color1 = color1;
        this.color2 = color2;
        this.color3 = color3;
        this.timer.start();
    }

     @Override
     
    protected void updateLEDs(AddressableLed buf, double time) {
        Color decidedColor;
        if (this.timer.get() < (time/3)) {
            decidedColor = color1;
        }

        else if (this.timer.get() > (time/3) && this.timer.get() < 2*time/3) {
            decidedColor = color2;
        }
        else {
          decidedColor = color3;
        }
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setLED(i, decidedColor);

        }
        
    }


}
    
    

