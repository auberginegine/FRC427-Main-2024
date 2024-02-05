package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AutomaticallyMoveToPiece {
    private PhotonCamera camera = new PhotonCamera("photonvision");

    public Command automaticallyMoveToPiece(Drivetrain drivetrain, Arm arm) {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Commands.none();
        }
        PhotonTrackedTarget target = result.getBestTarget();
        double angleToTurn = target.getYaw();

        return Commands.none(); 
    }
}
