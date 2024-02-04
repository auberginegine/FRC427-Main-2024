package frc.robot.subsystems.leds.patterns;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.leds.AddressableLed;

public class TestColorPattern extends LEDPattern {

    public TestColorPattern() {
        super(0);
        SmartDashboard.putNumber("color r", 0); 
        SmartDashboard.putNumber("color g", 0); 
        SmartDashboard.putNumber("color b", 0); 
    }

    @Override
    protected void updateLEDs(AddressableLed buf, double time) {
        double r = SmartDashboard.getNumber("color r", 0); 
        double g = SmartDashboard.getNumber("color g", 0); 
        double b = SmartDashboard.getNumber("color b", 0); 
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setRGB(i, (int) r, (int) g, (int) b);
        }
    }

}
