package frc.robot.subsystems.arm.commands;

import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

// controlled by pid 
public class GoToSpeaker extends GoToAngle {
    Arm m_arm;
    
    public GoToSpeaker(Arm arm) {
        super(arm, Constants.ArmConstants.kSpeakerPosition);
    }

}