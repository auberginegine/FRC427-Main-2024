package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleConfig {
    private int kDrive; 
    private int kRotate; 
    private int kEncoder; 
    private double kAbsOffset;
    private boolean kDriveInverted; 
    private SensorDirectionValue direction; 

    public SwerveModuleConfig(int kDrive, int kRotate, int kEncoder, double kAbsOffset, boolean kDriveInverted, SensorDirectionValue direction) {
        this.kDrive = kDrive; 
        this.kRotate = kRotate; 
        this.kEncoder = kEncoder;
        this.kAbsOffset = kAbsOffset; 
        this.kDriveInverted = kDriveInverted; 
        this.direction = direction; 
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

    public boolean getDriveInverted() {
        return this.kDriveInverted; 
    }

    public SensorDirectionValue getAbsoluteEncoderDirection() {
        return this.direction; 
    }
}
