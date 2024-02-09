// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutomationCommands;
import frc.robot.commands.ShootAnywhere;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlState;
import frc.robot.subsystems.arm.commands.GoToAmp;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.arm.commands.GoToTravel;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.MoveToAmp;
import frc.robot.subsystems.drivetrain.commands.TeleOpCommand;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.commands.SetHangSpeed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToAmp;
import frc.robot.subsystems.intake.commands.OuttakeToSpeaker;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.subsystems.intake.commands.SetSuckerIntakeSpeed;
import frc.robot.subsystems.intake.commands.TuneIntakeShooter;
import frc.robot.util.DriverController;
import frc.robot.util.DriverController.Mode;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.patterns.LEDPattern;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final AutoPicker autoPicker; 
  // private final SwerveTurnTunerCommand tunerCommand = new SwerveTurnTunerCommand(Constants.DrivetrainConstants.frontLeft);

  // drivetrain of the robot
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // intake of the bot
  private final Intake intake = Intake.getInstance(); 

  // leds!
  private final Led led = Led.getInstance(); 


  // limelight subsystem of robot
  private final Vision limelight = Vision.getInstance(); 

  // hang mechanism of robot
  private final Hang hang = Hang.getInstance();
  
  // arm of the robot
  private final Arm arm = Arm.getInstance();
  
  private SendableChooser<LEDPattern> patterns = new SendableChooser<>();
  
  
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

    patterns.addOption("Idle", Constants.LEDs.Patterns.kIdle);
    patterns.addOption("Rainbow", Constants.LEDs.Patterns.kBalanceFinished);
    patterns.addOption("Dead", Constants.LEDs.Patterns.kDead);
    patterns.addOption("Enabled", Constants.LEDs.Patterns.kEnabled);
    patterns.addOption("Disabled", Constants.LEDs.Patterns.kDisabled);
    patterns.addOption("Moving", Constants.LEDs.Patterns.kMoving);
    patterns.addOption("Failure", Constants.LEDs.Patterns.kFail);
    patterns.addOption("Intake", Constants.LEDs.Patterns.kIntake);
    patterns.addOption("Shooting", Constants.LEDs.Patterns.kShootAnywhere);
    patterns.addOption("Arm Travel", Constants.LEDs.Patterns.kArmMoving);
    patterns.addOption("Arm at amp", Constants.LEDs.Patterns.kArmAtAmp);
    patterns.addOption("Arm At Speaker", Constants.LEDs.Patterns.kArmAtSpeaker);
    patterns.addOption("Arm At Ground", Constants.LEDs.Patterns.kArmAtGround);
    patterns.addOption("Arm Moving", Constants.LEDs.Patterns.kArmCustom);
    patterns.addOption("Hanging", Constants.LEDs.Patterns.kHangActive);
    patterns.addOption("Auto Begins", Constants.LEDs.Patterns.kAutoBegin);
    patterns.addOption("Auto Ends", Constants.LEDs.Patterns.kAutoEnd);
    patterns.addOption("test Yellow", Constants.LEDs.Patterns.kCone);
    patterns.addOption("test Color", Constants.LEDs.Patterns.kTestColor);
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

    // --- Driver ---

    driverController.a().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

    // driverController.b().onTrue(new TuneTurnToAngle(drivetrain)); 
    // driverController.y().onTrue(new TuneBalance(drivetrain)); 

    driverController.rightTrigger()
      .onTrue(new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)))
      .onFalse(new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL))); 

    // new Trigger(() -> driverController.getRightY() > 0.5)
    //   .onTrue(new ())

   //  driverController.y().onTrue(new MoveToAmp(() -> drivetrain()));
   driverController.y()
   .onTrue(AutomationCommands.pathFindToAmpAndScore(arm, intake));


   driverController.b()
   .onTrue(AutomationCommands.pathFindToSpeakerAndScore(arm, intake));

   driverController.a()
   .onTrue(AutomationCommands.pathFindToGamePiece()); // make find to note

 

    // --- Intake --- 

    // outtake
    manipulatorController.leftBumper().and(() -> arm.getArmControlState() == ArmControlState.AMP)
      .onTrue(new OuttakeToAmp(intake));

    manipulatorController.leftBumper().and(() -> arm.getArmControlState() == ArmControlState.SPEAKER)
      .onTrue(new OuttakeToSpeaker(intake));
      
    // intake
    manipulatorController.rightBumper()
      .onTrue(new IntakeFromGround(intake));

    // intake from ground
   

    new Trigger(() -> manipulatorController.getRightY() > 0.5)
      .onTrue(AutomationCommands.shootFromAnywhere()); // TODO: make drivetrain thing

    new Trigger(() -> manipulatorController.getRightY() > -0.5 && manipulatorController.getRightY() < 0.5)
      .onTrue(new SetShooterSpeed(intake, 0));

      manipulatorController.rightTrigger()
      .onTrue(AutomationCommands.autoIntakeCommand()); // intake from ground auto

    // shoot
    new Trigger(() -> manipulatorController.getRightY() < -0.5) // outtake sucker
       .onTrue(new SetShooterSpeed(intake, Constants.IntakeConstants.kShootSpeed));

    // new Trigger(() -> manipulatorController.getRightY() < -0.5) // intake sucker
    //   .onTrue(new SetSuckerIntakeSpeed(intake, Constants.IntakeConstants.kSuckerManualSpeed));

    // new Trigger(() -> manipulatorController.getRightY() > 0.5)  // shoot out
    //   .onTrue(new SetShooterSpeed(intake, Constants.IntakeConstants.kShooterManualSpeed));

    // new Trigger(() -> manipulatorController.getRightY() < -0.5) // shoot in? (probably not needed)
    //   .onTrue(new SetShooterSpeed(intake, -Constants.IntakeConstants.kShooterManualSpeed));

    // new Trigger(() -> (manipulatorController.getRightY() <= 0.5 && manipulatorController.getRightY() >= -0.5)) 
    //   .onTrue(new SetSuckerIntakeSpeed(intake, 0))
    //   .onTrue(new SetShooterSpeed(intake, 0));


    // TODO: add automated controls for intaking from ground, outtaking to amp, outtaking to shooter
      
    // --- Arm ---

    // TODO: add manual control?
    // // right stick y to manually move arm
    // new Trigger(() -> manipulatorController.getLeftY() < -0.5) 
    //   .onTrue(new SetVelocity(arm, -Constants.ArmConstants.kTravelSpeed));
      
    // new Trigger(() -> (manipulatorController.getLeftY() <= 0.5 && manipulatorController.getLeftY() >= -0.5))
    //   .onTrue(new SetVelocity(arm, 0));

    // new Trigger(() -> manipulatorController.getLeftY() > 0.5)
    //   .onTrue(new SetVelocity(arm, Constants.ArmConstants.kTravelSpeed));
      
    // // buttons to move arm to go to setpoints
    manipulatorController.a().onTrue(new GoToTravel(arm));
    manipulatorController.b().onTrue(new GoToAmp(arm));
    manipulatorController.x().onTrue(new GoToSpeaker(arm));
    manipulatorController.y().onTrue(new GoToGround(arm));


    // --- Hang ---

    // Hang Up when DPAD UP
    manipulatorController.povUp()
      .onTrue(new SetHangSpeed(hang, Constants.HangConstants.kHangSpeed)); 
    //Hang Down when DPAD DOWN
    manipulatorController.povDown()
      .onTrue(new SetHangSpeed(hang, -Constants.HangConstants.kHangSpeed)); 

    // Stop hang when neither is pressed
    manipulatorController.povDown().negate().and(manipulatorController.povUp().negate())
    .onTrue(new SetHangSpeed(hang, 0)); 
  }
  

  // send any data as needed to the dashboard
  public void doSendables() {
    SmartDashboard.putData("Autonomous", autoPicker.getChooser());
    SmartDashboard.putBoolean("gyro connected", drivetrain.gyro.isConnected()); 
    SmartDashboard.putData(patterns);
    led.setPattern(patterns.getSelected());
  }

  // givess the currently picked auto as the chosen auto for the match
  public Command getAutonomousCommand() {
      // return null; 
    return autoPicker.getAuto();
    // return tunerCommand;

  }
}
