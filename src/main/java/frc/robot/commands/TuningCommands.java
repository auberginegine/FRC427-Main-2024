package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.util.IOUtils;

public class TuningCommands {
    public static Command tuneShootAngle(Arm arm, Intake intake) {

    return Commands.defer(() -> {
        double angle = IOUtils.get("arm angle"); 

        return Commands.sequence(
            new GoToAngle(arm, angle),
            OuttakeToSpeaker.outtakeToSpeaker(intake)
        );
    }, Set.of(arm, intake )); 


     //I wasn't hiding my code from the world...
     // the world was hiding it.

    }
    
}
