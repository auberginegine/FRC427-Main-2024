package frc.robot.subsystems.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlType;
import frc.robot.util.IOUtils;

public class TunePIDGoToAngle extends Command {
    Arm m_arm;

    public TunePIDGoToAngle(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    public void initialize() {
        m_arm.setControlType(ArmControlType.PID);
    }

    public void execute() {
        m_arm.setPID(IOUtils.get("arm p", Constants.ArmConstants.kP), 
                    IOUtils.get("arm i", Constants.ArmConstants.kI), 
                    IOUtils.get("arm d", Constants.ArmConstants.kD));

        m_arm.m_armFeedforward = new ArmFeedforward(IOUtils.get("arm ks", Constants.ArmConstants.kS), 
                                                    IOUtils.get("arm kg", Constants.ArmConstants.kG), 
                                                    IOUtils.get("arm kv", Constants.ArmConstants.kV), 
                                                    IOUtils.get("arm ka", Constants.ArmConstants.kA));
                                      
        m_arm.goToAngle(IOUtils.get("ArmAngle"));

        IOUtils.set("Current angle of Arm", m_arm.getAngle());
    }

    public boolean isFinished() {
        // confirmation that arm is at angle
        return m_arm.isAtAngle(); 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
