package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;
import frc.robot.util.IOUtils;

public class TuneGoToAngle extends Command {
    Arm m_arm;
    double m_angle;

    public TuneGoToAngle(Arm arm) {
        m_arm = arm;
        m_angle = IOUtils.get("ArmAngle");

        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.PID);
    }

    public void execute() {
    
        m_arm.goToAngle(m_angle);
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
