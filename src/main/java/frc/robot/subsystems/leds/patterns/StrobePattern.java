package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.util.Color;


public class StrobePattern extends TimedPattern {

    public StrobePattern(Color c, double strobeTime) {
        addPattern(new SolidLEDPattern(c), strobeTime);
        timedWait(strobeTime);
    }

}
