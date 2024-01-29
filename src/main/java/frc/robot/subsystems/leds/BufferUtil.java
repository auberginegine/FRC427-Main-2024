package frc.robot.subsystems.leds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class BufferUtil {
    //Sets each LED to the color input
    public static void setAll(AddressableLed buffer, Color color) {
        //Runs through each LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }
    //Sets each LED to the color 8bit color input
    public static void setAll(AddressableLed buffer, Color8Bit color) {
        //Runs through each LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
    }
    //Sets each LED to the RGB input
    public static void setAllRGB(AddressableLed buffer, int r, int g, int b) {
        //Runs through each LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
    //Sets each LED to the HSV input
    public static void setAllHSV(AddressableLed buffer, int h, int s, int v) {
        //Runs through each LED
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, h, s, v);
        }
    }
}
