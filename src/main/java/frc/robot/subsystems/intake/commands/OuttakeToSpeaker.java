package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class OuttakeToSpeaker extends SequentialCommandGroup {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // uses other commands to start the shooter first (cause its slow) then makes the sucker move a ring to the shooter after waiting a sec
    public OuttakeToSpeaker(Intake intake) {
        addCommands(
            new SetShooterSpeed(intake, Constants.IntakeConstants.kShootSpeed), 
            new WaitCommand(Constants.IntakeConstants.kShootRevTime), 
            new SetSuckerIntakeSpeed(intake, Constants.IntakeConstants.kShootSuckerSpeed),
            new WaitCommand(Constants.IntakeConstants.kShootWaitTime)
        );

        finallyDo(() -> {
            intake.stopShoot();
            intake.stopSuck();
        }); 

        addRequirements(intake);
    }
}
