package frc.robot.subsystems.arm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;

// not controlled by pid
public class SetVelocity extends Command {
    Arm m_arm;
    double m_velocity;

    public SetVelocity (Arm arm, double velocity) {
        m_arm = arm;
        m_velocity = velocity;

        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.MANUAL);
    }

    public void execute() {
        m_arm.setSpeed(m_velocity);
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }

}
