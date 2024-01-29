package frc.robot.subsystems.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

// controlled by pid 
public class GoToTravel extends GoToAngle {
    Arm m_arm;
    
    public GoToTravel(Arm arm) {
        super(arm, Constants.ArmConstants.kTravelPosition);
    }

}
