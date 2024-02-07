package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.vision.FrontVision;
import frc.robot.util.DriverController;

public class AutomaticallyMoveToPiece {

    public Command automaticallyMoveToPiece(DriverController driverController, Drivetrain drivetrain, FrontVision frontVision) {
        var result = frontVision.getLatestVisionResult();
        if (!result.hasTargets()) {
            return Commands.none();
        }
        
        double angleToTurn = frontVision.getNoteRotation();;

        return new ParallelCommandGroup(new TurnToAngle(drivetrain, angleToTurn).andThen(Commands.run(() -> {
            drivetrain.swerveDriveRobotCentric(new ChassisSpeeds(driverController.getLeftStickX(), 0, 0));
        }, drivetrain)), AutomationCommands.autoIntakeCommand()); // Any processing before turning to that angle

        // make command that feeds controller left y input to the swerve drive command
        // swerve drive robot centric
        // parallel race with auto intake command
    }
}
