package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTemplate extends Command {
    public CommandTemplate() {
    
    }

    public void initialize() {
        // runs when the command is FIRST STARTED
    }

    public void execute() {
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
