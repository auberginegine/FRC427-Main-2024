package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class OuttakeToShooter extends SequentialCommandGroup {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // uses other commands to start the shooter first (cause its slow) then makes the sucker move a ring to the shooter after waiting a sec
    public OuttakeToShooter(Intake intake, double shooterSpeed, double suckSpeed) {
        addCommands(
            new SetShooterSpeed(intake, suckSpeed), 
            new WaitCommand(1), 
            new SetSuckerSpeed(intake, suckSpeed),
            new WaitCommand(1)
        );

        finallyDo(() -> {
            intake.intakeRing(0);

        }); 

    addRequirements(intake);
    }
}
