package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IOUtils {
    public static double get(String key, double defaultVal) {
        if (!SmartDashboard.containsKey(key)) SmartDashboard.putNumber(key, defaultVal); 
        return SmartDashboard.getNumber(key, defaultVal); 
    }

    public static double get(String key) {
        return get(key, 0); 
    }

    public static void set(String key, double value) {
        SmartDashboard.putNumber(key, value); 
    }
    
}
