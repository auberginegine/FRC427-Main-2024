package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

 /*
  * functionality/specs/notes: 
  - one motor for each side of the intake (two motors in total)
  - make one of the motors the main motor and have the other one follow it
  - be able to intake at a specific speed
  - be able to outtake at a specific speed
  - be able to stop the motor (stop intaking/outtaking)
  */

public class Intake extends SubsystemBase {
    // put motors & stuff here
    CANSparkMax m_intakeMotorShootTop = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorShootTopId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderRight = m_intakeMotorShootTop.getEncoder();

    CANSparkMax m_intakeMotorShootBottom = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorShootBottomId, MotorType.kBrushless);
    RelativeEncoder m_intakeEncoderLeft = m_intakeMotorShootBottom.getEncoder();

    CANSparkMax m_outtakeMotorSuck = new CANSparkMax(Constants.IntakeConstants.kOuttakeMotorSuckId, MotorType.kBrushless);
    
    DigitalInput m_BeamBreak = new DigitalInput(Constants.IntakeConstants.kBeamBreakId); 

    
    public Intake() {
    }

    public void setupMotors() {
        m_intakeMotorShootTop.setInverted(Constants.IntakeConstants.kShootTopIntakeInverted);
        m_intakeMotorShootTop.setSmartCurrentLimit(Constants.IntakeConstants.kShootTopMotorlimit);

        m_intakeMotorShootBottom.setInverted(Constants.IntakeConstants.kShootBottomIntakeInverted);
        m_intakeMotorShootBottom.setSmartCurrentLimit(Constants.IntakeConstants.kShootBottomMotorlimit);
        m_intakeMotorShootBottom.follow(m_intakeMotorShootTop);

        m_outtakeMotorSuck.setInverted(Constants.IntakeConstants.kSuckOuttakeInverted);
        m_outtakeMotorSuck.setSmartCurrentLimit(Constants.IntakeConstants.kSuckOuttakeMotorLimit);

    }

    public void periodic() {
        // code inside here will run repeatedly while the robot is on
        
    }
    public void intakeRing(double speed){
        m_intakeMotorShootTop.set(-speed);
    }
    public void outtakeRing(double speed){
        m_outtakeMotorSuck.set(speed);
    }
    public boolean beamBreakHit(){ 
        return m_BeamBreak.get(); 
    }
    public void stopMotor(){
        m_intakeMotorShootTop.set(0);
    }
}