package frc.robot.subsystems.hang.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class SetHangSpeed extends Command{
    Hang m_Hang;
    double m_speed;
    
    public SetHangSpeed(Hang hang, double speed) {
        this.m_Hang = hang;
        this.m_speed = speed;
        addRequirements(hang);
    }


    public void initialize() {
        m_Hang.setSpeed(m_speed);
        // runs when the command is FIRST STARTED
    }

    public void execute() {
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return true; 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
