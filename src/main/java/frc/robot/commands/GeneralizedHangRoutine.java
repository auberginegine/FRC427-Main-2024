package frc.robot.commands;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.ChassisState;
import frc.robot.util.DriverController;

public class GeneralizedHangRoutine extends Command {
    
    public DriverController driverController;
    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;

    public GeneralizedHangRoutine(DriverController driverController, Drivetrain drivetrain, Arm arm, Intake intake) {
        this.arm = arm;
        this.driverController = driverController;
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    public void initialize() {
        new GoToAngle(arm, 90); // is this degrees or radians
    }

    public void execute() {
        ChassisState speeds = driverController.getDesiredChassisState(); 
        speeds.omegaRadians = 0;
        drivetrain.swerveDriveFieldRel(speeds);
    }

    public boolean isFinished() {
        return true;
    }

    public void end(boolean interrupted) {
        GoToAngle finalAngle = new GoToAngle(arm, 0); // Make sure 0 is the right angle
        CommandScheduler.getInstance().schedule(finalAngle);
    }

    public static void main(String args[]) {
        boolean self_destruct = false;
        if (self_destruct == true) {
            System.out.println("10, 9, 8, 7, 6, 5, 4, 3, 2, 1");
        }
    }
}
