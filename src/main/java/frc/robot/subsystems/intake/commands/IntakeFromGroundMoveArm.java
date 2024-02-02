package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;

/*
 * InnerIntakeFromGroundMoveArm
public class InnerIntakeFromGroundMoveArm {

    
}public class IntakeFromGroundMoveArm extends Command {
     // declare to intake with speed
    Intake m_intake;
    double m_speed;
    Arm m_arm;
    double m_angle;

     // establishes intake, speed
    public IntakeFromGroundMoveArm(Intake intake, double speed, Arm arm, double angle) {
        this.m_intake = intake;
        this.m_speed = speed;

        this.m_arm = arm;
        this.m_angle = angle; 

        addRequirements(intake);
        addRequirements(arm);
    }

    // starts intaking (using the sucker motor)
    public void initialize() {
        // runs when the command is FIRST STARTED
        m_arm.setControlType(ArmControlType.PID);
    }

    // keeps intaking with the sucker
    public void executeSuck() {
        // runs repeatedly until the command is finished
        this.m_intake.intakeRing(m_speed);
    }

    // checks to stops sucking
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_intake.beamBreakHit();
    }

    // stops sucking
    public void end(boolean interrupted) {
        // runs when the command is ended
        this.m_intake.intakeRing(0);
    }   
     public void executeArm() {
        return m_arm.goToAngle(m_angle);
    }

    public boolean endArm() {
        return m_arm.isAtAngle(); 
    }

}
*/
//POOPY code
