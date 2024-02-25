package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAngle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.util.GeometryUtils;
import frc.robot.util.quad.AngleBisector;
import frc.robot.util.quad.OrderedPair;

public class ShootAnywhereWithBisector {

    public static Command shootAnywhereWithBisector(Drivetrain drivetrain, Arm arm, Intake intake) {
        return shootAnywherewithBisector(drivetrain, arm, intake, 1); 
    }
    public static Command shootAnywherewithBisector(Drivetrain drivetrain, Arm arm, Intake intake, double speed) {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d targetPose = null;

        Optional<Alliance> optAlliance = DriverStation.getAlliance(); 

        if (optAlliance.isEmpty()) return Commands.none();

        Alliance alliance = optAlliance.get();
        if (alliance == DriverStation.Alliance.Blue) {
            OrderedPair angleBisector = GeometryUtils.getBisector(new OrderedPair(currentPose.getX(), currentPose.getY()), Constants.Vision.kBlueAllianceLeftSpeakerCoordinate, Constants.Vision.kBlueAllianceRightSpeakerCoordinate);
            targetPose = new Pose2d(angleBisector.getX(), angleBisector.getY(), new Rotation2d());
        }
        else if (alliance == DriverStation.Alliance.Red) {
            OrderedPair angleBisector = GeometryUtils.getBisector(new OrderedPair(currentPose.getX(), currentPose.getY()), Constants.Vision.kRedAllianceLeftSpeakerCoordinate, Constants.Vision.kRedAllianceRightSpeakerCoordinate);
            targetPose = new Pose2d(angleBisector.getX(), angleBisector.getY(), new Rotation2d());        }
        if (targetPose == null) return Commands.none();

        double finalAngle = Math.atan2(currentPose.getY() - targetPose.getY(),  currentPose.getX() - targetPose.getX());
        double distance = Math.hypot(currentPose.getY() - targetPose.getY(), currentPose.getX() - targetPose.getX()); 
        TurnToAngle turnToAngle = new TurnToAngle(drivetrain, Math.toDegrees(finalAngle));
        double angleToTurnArm = Constants.GeneralizedReleaseConstants.distanceToArmAngle.apply(distance);
        GoToAngle goToAngle = new GoToAngle(arm, angleToTurnArm);
        // Command outtake = OuttakeToSpeaker.outtakeToSpeaker(intake);
        Command rev = OuttakeToSpeaker.revAndIndex(intake); 
        Command shoot = OuttakeToSpeaker.shoot(intake, 0.5); 
        SmartDashboard.putNumber("Shoot Anywhere Arm Angle", angleToTurnArm); 
        SmartDashboard.putNumber("Shoot Anywhere Distance", distance); 
        return Commands.sequence(Commands.parallel(turnToAngle, goToAngle, rev), shoot)
        .finallyDo(() -> {
            arm.goToAngle(Constants.ArmConstants.kTravelPosition);
        });
    }
}
