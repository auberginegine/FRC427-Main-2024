package frc.robot.subsystems.hang;



import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class Hang extends SubsystemBase {
    private double m_velocity = 0;
    
    //Initialize Motors
    private CANSparkMax m_HangMotorRight = new CANSparkMax(Constants.HangConstants.kHangRightMotorID, MotorType.kBrushless);
    private CANSparkMax m_HangMotorLeft = new CANSparkMax(Constants.HangConstants.kHangLeftMotorID, MotorType.kBrushless);

    //Encoder Initialize
    private RelativeEncoder m_HangEncoderRight = m_HangMotorRight.getEncoder();
    private RelativeEncoder m_HangEncoderLeft = m_HangMotorLeft.getEncoder();



    public Hang() {
        setupMotors();
    }

    public void setupMotors() {

        m_HangMotorRight.setInverted(Constants.HangConstants.kRightMotorInverted);
        m_HangMotorLeft.setInverted(Constants.HangConstants.kLeftMotorInverted);
        
        m_HangMotorRight.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);
        m_HangMotorLeft.setSmartCurrentLimit(Constants.HangConstants.kHangMotorLimit);

        m_HangEncoderRight.setPositionConversionFactor(Constants.HangConstants.kPositionConversionFactor);
        m_HangEncoderRight.setVelocityConversionFactor(Constants.HangConstants.kVelocityConversionFactor);
        
        //Sets  limits for Right and Left Motors
        m_HangMotorRight.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        m_HangMotorRight.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_HangMotorRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kForward, Constants.HangConstants.kFowardHangSoftLimit);
        m_HangMotorLeft.setSoftLimit(SoftLimitDirection.kReverse, Constants.HangConstants.kReverseHangSoftLimit);
        m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_HangMotorLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

        //Tells Left Motor to do whatever right motor is doing
        m_HangMotorLeft.follow(m_HangMotorRight);

}
    public void periodic() {
        //Constantly setting speed to whatever velocity is
        m_HangMotorRight.set(m_velocity);
        
        

    }

    public void setSpeed(double speed) {
        //Method to change speed
        this.m_velocity = speed;
    }

}