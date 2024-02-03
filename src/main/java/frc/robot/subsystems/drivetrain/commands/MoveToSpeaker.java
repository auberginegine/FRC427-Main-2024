package frc.robot.subsystems.drivetrain.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class MoveToSpeaker {
    public static Pose2d getTargetPose() {
        Optional<Alliance> optAlliance = DriverStation.getAlliance();

        if (optAlliance.isEmpty()) return new Pose2d();

        Alliance alliance = optAlliance.get();

        Pose2d estPose = Drivetrain.getInstance().getPose(); 

        Pose2d TargetPose = new Pose2d();

        if (alliance == Alliance.Blue) {
            TargetPose = estPose.nearest(List.of(
                Constants.PathFollower.speakerBlue1,
                Constants.PathFollower.speakerBlue2,
                Constants.PathFollower.speakerBlue3

            ));
        }
        if (alliance == Alliance.Red) {
            TargetPose = estPose.nearest(List.of(
                Constants.PathFollower.speakerRed1,
                Constants.PathFollower.speakerRed2,
                Constants.PathFollower.speakerRed3
            ));
        }

        return TargetPose;

    }

    public static Command GoToSpeaker() {
        Pose2d targetPose = getTargetPose();

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );
        return pathfindingCommand;
    }
}
