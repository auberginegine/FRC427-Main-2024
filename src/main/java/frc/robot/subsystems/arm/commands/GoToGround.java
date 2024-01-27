package frc.robot.subsystems.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

// controlled by pid 
public class GoToGround extends GoToAngle {
    Arm m_arm;
    
    public GoToGround(Arm arm) {
        super(arm, Constants.ArmConstants.kGroundPosition);
    }

}