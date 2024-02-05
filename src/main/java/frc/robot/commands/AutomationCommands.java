package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.MoveToAmp;
import frc.robot.subsystems.drivetrain.commands.MoveToSpeaker;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;

public class AutomationCommands {
    
  public static Command autoIntakeCommand() {
    return Commands.sequence(
        new GoToGround(Arm.getInstance()), 
        new IntakeFromGround(Intake.getInstance())
    ).finallyDo(() -> {
      Arm.getInstance().goToAngle(Constants.ArmConstants.kTravelPosition);
    });
  }

  public static Command pathFindToSpeaker() {
    return Commands.defer(() -> MoveToSpeaker.goToSpeaker(), Set.of(Drivetrain.getInstance())); 
  }

  public static Command pathFindToAmp() {
    return Commands.defer(() -> MoveToAmp.goToAmp(), Set.of(Drivetrain.getInstance())); 
  }

  public static Command pathFindToSpeakerAndScore() {
    // TODO
    return Commands.none(); 
  }

  public static Command pathFindToAmpAndScore() {
    // TODO
    return Commands.none(); 
  }

  public static Command pathFindToGamePiece() {
    // TODO
    return Commands.none(); 
  }

  public static Command shootFromAnywhere() {
    return Commands.defer(
      () -> ShootAnywhere.shootAnywhere(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()), 
    Set.of(Drivetrain.getInstance(), Arm.getInstance(), Intake.getInstance()));  
  }
}
