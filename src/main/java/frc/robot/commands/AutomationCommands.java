package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToAmp;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.MoveToAmp;
import frc.robot.subsystems.drivetrain.commands.MoveToSpeaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToAmp;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.leds.Led;

public class AutomationCommands {
    
  public static Command autoIntakeCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> Led.getInstance().isIntaking = true),
        new GoToGround(Arm.getInstance()), 
        new IntakeFromGround(Intake.getInstance())
    ).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
      Commands.runOnce(() -> Led.getInstance().isIntaking = false);
    });
  }

  public static Command pathFindToSpeaker() {
    return Commands.runOnce(() -> Led.getInstance().isMovingToSpeaker = true).andThen(
      Commands.defer(() -> MoveToSpeaker.goToSpeaker(), Set.of(Drivetrain.getInstance()))).finallyDo(() -> {
        Led.getInstance().isMovingToSpeaker = false;
      }); 
  }

  public static Command pathFindToAmp() {
    return Commands.runOnce(() -> Led.getInstance().isMovingToAmp = true).andThen(
      Commands.defer(() -> MoveToAmp.goToAmp(), Set.of(Drivetrain.getInstance()))).finallyDo(() -> {
        Led.getInstance().isMovingToAmp = false;
      }); 
  }

  public static Command pathFindToSpeakerAndScore(Arm arm, Intake intake) {
    return pathFindToSpeaker().alongWith(new GoToSpeaker(arm)).andThen(OuttakeToSpeaker.outtakeToSpeaker(intake)).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    }); 
  }

  public static Command pathFindToAmpAndScore(Arm arm, Intake intake) {
    return pathFindToAmp().alongWith(new GoToAmp(arm)).andThen(new OuttakeToAmp(intake)).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    });
  }

  public static Command pathFindToGamePiece() {
    // TODO
    return Commands.none(); 
  }

  public static Command shootFromAnywhere() {
    return Commands.runOnce(() -> Led.getInstance().isShooting = true).andThen(Commands.defer(
      () -> ShootAnywhere.shootAnywhere(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()), 
    Set.of(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()))).finallyDo(() -> {
      Led.getInstance().isShooting = false;
    }); 
  }
}
