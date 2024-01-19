package frc.robot.subsystems.drivetrain.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;

// command to tune the drive speed PID of a single swerve module
public class SwerveDriveTunerCommand extends Command {
    SwerveModule module; 

    /***
     * 
     * @param turn the CAN ID of the turn motor
     * @param drive the CAN ID of the drive motor
     * @param encoder the CAN ID of the turn encoder
     */
    public SwerveDriveTunerCommand(SwerveModuleConfig config) {
        this.module = new SwerveModule(config); 
    }

    @Override
    public void initialize() {
        // initialize the drive PID values of the module to the dashboard
        SmartDashboard.putNumber("SwerveDrivekP", SmartDashboard.getNumber("SwerveDrivekP", 0));
        SmartDashboard.putNumber("SwerveDrivekI", SmartDashboard.getNumber("SwerveDrivekI", 0));
        SmartDashboard.putNumber("SwerveDrivekD", SmartDashboard.getNumber("SwerveDrivekD", 0)); 
        SmartDashboard.putNumber("SwerveDriveSetpoint", SmartDashboard.getNumber("SwerveDriveSetpoint", 0)); 
    }

    @Override
    public void execute() {
        // set PID values periodically
        module.setDrivePID(
            SmartDashboard.getNumber("SwerveDrivekP", Constants.DrivetrainConstants.kModuleDrive_P), 
            SmartDashboard.getNumber("SwerveDrivekI", 0), 
            SmartDashboard.getNumber("SwerveDrivekD", 0), 
            SmartDashboard.getNumber("SwerveDrivekFF", 0)
            );

        // drive the module with the desired drive speed setpoint (from dashboard)
        module.updateState(new SwerveModuleState(SmartDashboard.getNumber("SwerveDriveSetpoint", 0), Rotation2d.fromDegrees(0)), SwerveModule.DriveState.CLOSED_LOOP);

        SmartDashboard.putNumber("SwerveTurnMeasurement", module.getCurrentState().speedMetersPerSecond); 
        SmartDashboard.putNumber("SwerveError", module.turnPIDController.getPositionError()); 
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
