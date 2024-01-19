package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

// class to store, set up, and choose autos
public class AutoPicker {
    private SendableChooser<Command> chooser; 

    Drivetrain m_driveSubsystem;
    
    public AutoPicker(Drivetrain driveSubsystem) {
        m_driveSubsystem = driveSubsystem;

        // addRequirements(driveSubsystem);

        // see PathPlanner

        AutoBuilder.configureHolonomic(
            driveSubsystem::getPose, // Pose2d supplier
            driveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            driveSubsystem::getChassisSpeeds, 
            driveSubsystem::swerveDrive, // Module states consumer used to output to the drive subsystem
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.Trajectory.kDrive_P, Constants.Trajectory.kDrive_I, Constants.Trajectory.kDrive_D), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(Constants.Trajectory.kOmega_P, Constants.Trajectory.kOmega_I, Constants.Trajectory.kOmega_D), 
                Constants.DrivetrainConstants.kMaxAttainableModuleSpeedMetersPerSecond, Constants.DrivetrainConstants.kDriveBaseRadius, 
                new ReplanningConfig()
            ),  
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            driveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        );

        registerCommands(); 


        chooser = AutoBuilder.buildAutoChooser(); 
    }

    public void registerCommands() {
        // eg. NamedCommands.registerCommand("intake_cube", new IntakeForTime(intake, 1, 2)); 

    }

    // gets the currently selected auto
    public Command getAuto() {
        return chooser.getSelected(); 
    }

    public SendableChooser<Command> getChooser() {
        return chooser; 
    }
}
