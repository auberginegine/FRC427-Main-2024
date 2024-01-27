package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AddressableLed {
    //Create Buffer, and start/end
    private AddressableLEDBuffer buffer; 
    private int start; 
    private int end; 

    public AddressableLed(AddressableLEDBuffer buffer, int start, int end) {
        //Reassigns buffer, start/end
        this.buffer = buffer; 
        this.start = start; 
        this.end = end; 
    }
    
    public AddressableLed getSection(int start, int end) {
        return new AddressableLed(buffer, getOffsetIndex(start), getOffsetIndex(end)); 
    }

    //Gets Index
    private int getOffsetIndex(int i) {
        int index = start + i; 
        if (index > end) throw new ArrayIndexOutOfBoundsException(); 
        return index; 
    }
    //Sets color with RGB
    public void setRGB(int i, int r, int g, int b) {
        buffer.setRGB(getOffsetIndex(i), r, g, b);
    }
    //Sets color with HSV
    public void setHSV(int i, int h, int s, int v) {
        buffer.setHSV(getOffsetIndex(i), h, s, v);
    }

    //Sets color with ... color
    public void setLED(int i, Color color) {
        buffer.setLED(getOffsetIndex(i), color);
    }

    //Sets color with ... 8bitcolor
    public void setLED(int i, Color8Bit color) {
        buffer.setLED(getOffsetIndex(i), color);
    }

    //Gets length of leds
    public int getLength() {
        return this.end - this.start; 
    }

    //gets start Led
    public int getStart() {
        return start; 
    }
    //Gets end Led
    public int getEnd() {
        return end; 
    }
}
