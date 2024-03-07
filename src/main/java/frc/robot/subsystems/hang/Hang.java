package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.UUIDGenerator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.IOUtils;

public class Hang extends SubsystemBase {
    private static Hang instance = new Hang();

    public static Hang getInstance() {
        return instance; 
    }

    //Initializing velocity variable
    public double m_targetPosition;
    
    //Initialize Motors
    private CANSparkMax m_HangMotor = new CANSparkMax(Constants.HangConstants.kHangMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_HangEncoder = m_HangMotor.getEncoder();

    private SparkPIDController m_HangPIDController = m_HangMotor.getPIDController(); 

    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotor.setInverted(Constants.HangConstants.kMotorInverted);
        
        //Sets Smart Limits
        m_HangMotor.setSmartCurrentLimit(20, Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for encoders
        m_HangEncoder.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoder.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotor.setIdleMode(IdleMode.kBrake);

        setPID(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);
        
        m_HangMotor.burnFlash();
    }

    public void setPID(double p, double i, double d) {
        m_HangPIDController.setP(p);
        m_HangPIDController.setI(i);
        m_HangPIDController.setD(d);
    }

    @Override
    public void periodic() {
        // m_HangPIDController.setReference(m_targetPosition, ControlType.kPosition); 

        //Constantly sends logs to Smart Dashboard
        doSendables();
    }

    public void setSpeed(double speed) {
        m_HangMotor.set(speed);
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Current Velocity", m_HangEncoder.getVelocity());
        IOUtils.set("Hang Current Position", m_HangEncoder.getPosition());
        IOUtils.set("Hang Target Position", m_targetPosition);
        IOUtils.set("Hang Error", getError());
        IOUtils.set("Hang Current", m_HangMotor.getOutputCurrent());
        IOUtils.set("Hang Bus Voltage", m_HangMotor.getBusVoltage());
        IOUtils.set("Hang Output", m_HangMotor.getAppliedOutput());
        
    }

    public double getHangPosition() {
        return m_HangEncoder.getPosition();
    }

    public void setPosition(double targetPosition) {
        this.m_targetPosition = targetPosition;
    }

    public boolean isAtPosition() {
        return getError() <= Constants.HangConstants.kHangTolerance; 
    }

    public double getError() {
        return Math.abs(m_HangEncoder.getPosition() - this.m_targetPosition); 
    }

}