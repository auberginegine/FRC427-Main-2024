package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OuttakeToAmp extends Command {
     // declare how long to intake for and speed
    Intake m_intake;
    double m_speed;

     // establishes intake, speed,
    public OuttakeToAmp(Intake intake, double speed) {
    this.m_intake = intake;
    this.m_speed = speed;

    addRequirements(intake);
    }

    // starts outtaking the ring from the sucker into the shooter 
    public void initialize() {
        // runs when the command is FIRST STARTED
    this.m_intake.outtakeRing(m_speed);
    this.m_intake.intakeRing(m_speed);
    }

    // keeps outtaking. The ring has probably gotten into the shooters now. 
    public void execute() {
        // runs repeatedly until the command is finished
    this.m_intake.outtakeRing(m_speed);
    this.m_intake.intakeRing(m_speed);
    }

    // checks to stops outtaking the ring
    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return m_intake.beamBreakState();
    }

    // stops outtaking ring
    public void end(boolean interrupted) {
        // runs when the command is ended
    this.m_intake.outtakeRing(0);
    }    
}
