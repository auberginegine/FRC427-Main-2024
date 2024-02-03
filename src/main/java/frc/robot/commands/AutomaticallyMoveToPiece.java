package frc.robot.commands;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;

public class AutomaticallyMoveToPiece extends SequentialCommandGroup{
    private PhotonCamera camera = new PhotonCamera("photonvision");
    private Drivetrain drivetrain;
    private Arm arm;

    public AutomaticallyMoveToPiece(Drivetrain drivetrain, Arm arm) {
             
        this.drivetrain = drivetrain;
        this.arm = arm;

        addRequirements(drivetrain, arm);
    }

    public Command automaticallyMoveToPiece() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Commands.none();
        }
        PhotonTrackedTarget target = result.getBestTarget();
        double angleToTurn = target.getYaw();

        return Commands.sequence(null);
    }
}
