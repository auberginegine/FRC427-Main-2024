package frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.leds.Led;
import frc.robot.util.IOUtils;

public class DriverCommands {
  public static Command vibrateController(GenericHID controller, double timeSeconds) {
    return Commands.runOnce(() -> {
      controller.setRumble(RumbleType.kBothRumble, 1);
    }).andThen(new WaitCommand(timeSeconds)).andThen(() -> { 
        controller.setRumble(RumbleType.kBothRumble, 0);
    });  
  }

  public static Command indicateBeamBreak(GenericHID controller) {
    if (DriverStation.isAutonomous()) return Commands.none();
    return Commands.parallel(
      Commands.runOnce(() -> {
        Led.getInstance().beamHit = true; 
      }), 
      vibrateController(controller, 0.5)
    )
    .andThen(new WaitCommand(3))
    .finallyDo(() -> Led.getInstance().beamHit = false); 
  }

  public static Command tuneShooting(Drivetrain drivetrain, Arm arm, Intake intake) {
    return Commands.defer(() -> {
        double angle = IOUtils.get("arm angle"); 

        Pose2d currentPose = drivetrain.getPose();
        Pose2d targetPose = null;

        Optional<Alliance> optAlliance = DriverStation.getAlliance(); 

        if (optAlliance.isEmpty()) return Commands.none();

        Alliance alliance = optAlliance.get();
        if (alliance == DriverStation.Alliance.Blue) {
            targetPose = Constants.GeneralizedReleaseConstants.kBlueAllianceSpeaker;
        }
        else if (alliance == DriverStation.Alliance.Red) {
            targetPose = Constants.GeneralizedReleaseConstants.kRedAllianceSpeaker;
        }
        if (targetPose == null) return Commands.none();

        double finalAngle = Math.atan2(currentPose.getY() - targetPose.getY(),  currentPose.getX() - targetPose.getX());
        double distance = Math.hypot(currentPose.getY() - targetPose.getY(), currentPose.getX() - targetPose.getX()); 
        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, Math.toDegrees(finalAngle));

        SmartDashboard.putNumber("TuneShooting/distance", distance); 
        SmartDashboard.putNumber("TuneShooting/angleToSpeaker", Math.toDegrees(finalAngle)); 

        return Commands.sequence(
            turnToAngle, 
            new GoToAngle(arm, angle),
            OuttakeToSpeaker.revAndIndex(intake),
            OuttakeToSpeaker.shoot(intake)
        );
    }, Set.of(drivetrain, arm, intake)); 
  }
}
