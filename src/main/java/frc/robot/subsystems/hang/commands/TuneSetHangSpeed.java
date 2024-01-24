package frc.robot.subsystems.hang.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;
import frc.robot.util.IOUtils;

public class TuneSetHangSpeed extends Command{
    Hang m_Hang;
    double m_speed;

    public TuneSetHangSpeed(Hang hang) {
        m_Hang = hang;
        
        addRequirements(hang);
    }


    public void initialize() {

        // runs when the command is FIRST STARTED
    }

    public void execute() {
        
        m_Hang.setSpeed(IOUtils.get("Speed"));
        // runs repeatedly until the command is finished
    }

    public boolean isFinished() {
        // runs and tells whether or not the command should finish
        return false; 
    }

    public void end(boolean interrupted) {
        // runs when the command is ended
    }
}
