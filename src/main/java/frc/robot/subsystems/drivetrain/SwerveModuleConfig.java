package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleConfig {
    private int kDrive; 
    private int kRotate; 
    private int kEncoder; 
    private double kAbsOffset;
    private boolean kRotateInverted;
    private boolean kDriveInverted; 
    private SensorDirectionValue direction;
    private String name;  

    public SwerveModuleConfig(String name, int kDrive, int kRotate, int kEncoder, double kAbsOffset, boolean kRotateInverted, boolean kDriveInverted, SensorDirectionValue direction) {
        this.kDrive = kDrive; 
        this.kRotate = kRotate; 
        this.kEncoder = kEncoder;
        this.kAbsOffset = kAbsOffset; 
        this.kRotateInverted = kRotateInverted;
        this.kDriveInverted = kDriveInverted; 
        this.direction = direction; 
        this.name = name; 
    }

    public String getName() {
        return this.name; 
    }

    public int getDriveId() {
        return this.kDrive; 
    }

    public int getRotateId() {
        return this.kRotate; 
    }

    public int getEncoderId() {
        return this.kEncoder; 
    }

    public double getAbsoluteEncoderOffset() {
        return this.kAbsOffset; 
    }

    public boolean getRotateInverted() {
        return this.kRotateInverted;
    }

    public boolean getDriveInverted() {
        return this.kDriveInverted; 
    }

    public SensorDirectionValue getAbsoluteEncoderDirection() {
        return this.direction; 
    }
}
