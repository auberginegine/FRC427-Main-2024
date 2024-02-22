package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.IOUtils;

public class Hang extends SubsystemBase {
    private static Hang instance = null; //new Hang();

    public static Hang getInstance() {
        return instance; 
    }

    //Initializing velocity variable
    private double m_velocity = 0;
    public double m_targetPosition;
    
    //Initialize Motors
    private CANSparkMax m_HangMotorRight = new CANSparkMax(Constants.HangConstants.kHangRightMotorID, MotorType.kBrushless);
    private CANSparkMax m_HangMotorLeft = new CANSparkMax(Constants.HangConstants.kHangLeftMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_HangEncoderRight = m_HangMotorRight.getEncoder();
    private RelativeEncoder m_HangEncoderLeft = m_HangMotorLeft.getEncoder();

    private PIDController m_HangPidController = new PIDController(Constants.HangConstants.kP, Constants.HangConstants.kI, Constants.HangConstants.kD);


    private Hang() {
        setupMotors();
    }

    public void setupMotors() {
        //Sets motors inverted
        m_HangMotorRight.setInverted(Constants.HangConstants.kRightMotorInverted);
        // m_HangMotorLeft.setInverted(Constants.HangConstants.kLeftMotorInverted);
        
        //Sets Smart Limits
        m_HangMotorRight.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);
        m_HangMotorLeft.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);

        //Conversion Factors for left Encoders
        m_HangEncoderLeft.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoderLeft.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        //Conversion Factors for right Encoders
        m_HangEncoderRight.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoderRight.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);

        m_HangMotorLeft.setIdleMode(IdleMode.kBrake);
        m_HangMotorRight.setIdleMode(IdleMode.kBrake);

        m_HangPidController.setTolerance(Constants.HangConstants.kHangTolerance);
        //Sets  limits for Right and Left Motors
        //Right Motors
        m_HangMotorRight.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        m_HangMotorRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        //Left Motors
        m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_HangPidController.setP(Constants.HangConstants.kP);
        m_HangPidController.setI(Constants.HangConstants.kI);
        m_HangPidController.setD(Constants.HangConstants.kD);
        

        // Tells Left Motor to do whatever right motor is doing
        m_HangMotorLeft.follow(m_HangMotorRight, Constants.HangConstants.kLeftMotorInverted);

        m_HangMotorRight.burnFlash();
        m_HangMotorLeft.burnFlash();
    }

    @Override
    public void periodic() {
        m_velocity = m_HangPidController.calculate(getHangPosition(), m_targetPosition);
        m_HangMotorRight.set(m_velocity);
       

        //Check for LEDs on Hang
        

        //Constantly sends logs to Smart Dashboard
        doSendables();
    }

    public void doSendables() {
        // Add logging for hang (eg. encoder positions, velocity, etc. )
        IOUtils.set("Hang Target Velocity (m/s)", m_velocity);
        IOUtils.set("Right Hang Current Velocity (m/s)", m_HangEncoderRight.getVelocity());
        IOUtils.set("Left Hang Current Velocity (m/s)", m_HangEncoderLeft.getVelocity());
        IOUtils.set("Right Hang Current Position", m_HangEncoderRight.getPosition());
        IOUtils.set("Left Hang Current Position", m_HangEncoderLeft.getPosition());

        // SmartDashboard.putBoolean("Hang RM Inverted", m_HangMotorRight.getInverted());
        // SmartDashboard.putBoolean("Hang LM Inverted", m_HangMotorLeft.getInverted());
        // IOUtils.set("Hang Soft Limit ForwardRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackRM", m_HangMotorRight.getSoftLimit(SoftLimitDirection.kReverse));
        // IOUtils.set("Hang Soft Limit ForwardLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kForward));
        // IOUtils.set("Hang Soft Limit BackLM", m_HangMotorLeft.getSoftLimit(SoftLimitDirection.kReverse));
        
    }

    public void setSpeed(double speed) {
        //Method to change speed
        this.m_velocity = speed;
    }

    public double getHangPosition() {
        return m_HangEncoderRight.getPosition();
    }

    public void setPosition(double targetposition) {
        this.m_targetPosition = targetposition;
    }

    public boolean isAtPosition() {
        return m_HangPidController.atSetpoint();
    }

  
}