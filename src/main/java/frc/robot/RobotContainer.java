// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TeleOpCommand;
import frc.robot.subsystems.intake.commands.SetIntakeSpeed;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.util.DriverController;
import frc.robot.util.DriverController.Mode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoPicker autoPicker; 

  // drivetrain of the robot
  private final Drivetrain drivetrain = new Drivetrain();
  
 //  public Command tunegotoangle2 = new TuneGoToAngle(arm);

  // controller for the driver
  private final DriverController driverController =
      new DriverController(0);

  private final CommandXboxController manipulatorController = new CommandXboxController(1); 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoPicker = new AutoPicker(drivetrain); 
    // Configure the trigger bindings
    configureBindings();

    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    // default command for drivetrain is to calculate speeds from controller and drive the robot
    drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverController));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driverController.a().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    // driverController.b().onTrue(new TuneTurnToAngle(drivetrain)); 
    // driverController.y().onTrue(new TuneBalance(drivetrain)); 

    driverController.rightTrigger()
      .onTrue(new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)))
      .onFalse(new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL))); 

//both Suck and Shoot have teh same controls RN (CHANGE ONCE DRIVERS TELL U WHAT CONTROLS THEY WANT)
    new Trigger(() -> manipulatorController.getRightY() > 0.5) 
      .onTrue(new SetSuckerSpeed(intake, -1));

    new Trigger(() -> manipulatorController.getRightY() < -0.5) 
      .onTrue(new SetSuckerSpeed(intake, 1));

    new trigger(() -> manipulatorController.getRightY() > 0.5) 
      .onTrue(new SetShooterSpeed(intake, 1));

    new trigger(() -> manipulatorController.getRightY() < -0.5) 
      .onTrue(new SetShooterSpeed(intake, -1));

    new Trigger(() -> (manipulatorController.getRightY() <= 0.5 && manipulatorController.getRightY() >= -0.5)) 
      .onTrue(new SetSuckereSpeed(intake, 0));
      .onTrue(new SetShooterSpeed(intake, 0));
      

//both Suck and Shoot have teh same controls RN (CHANGE ONCE DRIVERS TELL U WHAT CONTROLS THEY WANT)
  }
  // send any data as needed to the dashboard
  public void doSendables() {
    SmartDashboard.putData("Autonomous", autoPicker.getChooser());
    SmartDashboard.putBoolean("gyro connected", drivetrain.gyro.isConnected()); 
  }

  // givess the currently picked auto as the chosen auto for the match
  public Command getAutonomousCommand() {
    //  return null; 
    return autoPicker.getAuto();
  }
  
}
