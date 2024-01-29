package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.leds.AddressableLed;
import frc.robot.subsystems.leds.BufferUtil;

public abstract class LEDPattern {
    
    private double loopTime; 

    public LEDPattern(double loopTime) {
        this.loopTime = loopTime;
    }

    protected abstract void updateLEDs(AddressableLed buffer, double time);
    
    public void update(AddressableLed buffer, double time) {
        updateLEDs(buffer, loopTime == 0 ? time : time % loopTime);
    }

    public double getLoopTime() {
        return this.loopTime; 
    }

    public void setLoopTime(double loopTime) {
        this.loopTime = loopTime; 
    }

    public static class EmptyLEDPattern extends LEDPattern {
        public EmptyLEDPattern() {
            super(1); 
        }

        @Override
        protected void updateLEDs(AddressableLed buffer, double time) {
            BufferUtil.setAll(buffer, Color.kBlack);
        }
    }

    public static final LEDPattern kEmpty = new EmptyLEDPattern();  
}