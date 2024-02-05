package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;

public class SineLEDPattern extends LEDPattern {

    private final Color c1; 
    private final Color c2; 
    private final double w;
      
    public SineLEDPattern(double loopTime, Color c1, Color c2, double w) {
        super(loopTime);
        this.c1 = c1; 
        this.c2 = c2; 
        this.w = w;
    }

    @Override
    protected void updateLEDs(AddressableLed buf, double time) {
        // System.out.println(time);

         

        for (int i = 0; i < buf.getLength(); i++) {
            double interpolateVal = 0.5*Math.sin(Math.PI*i/w-2*Math.PI*time/getLoopTime())+0.5;
        // System.out.println(0.5*Math.cos(2*Math.PI/getLoopTime() * time));

        // boolean goTo = Math.cos(2*Math.PI/getLoopTime() * time) >= 0; 

            double r = MathUtil.interpolate(c2.red, c1.red, interpolateVal); 
            double g = MathUtil.interpolate(c2.green, c1.green, interpolateVal); 
            double b = MathUtil.interpolate(c2.blue, c1.blue, interpolateVal);

            buf.setRGB(i, (int) (r * 255), (int) (g * 255), (int) (b * 255));
        }
    }
    
}
