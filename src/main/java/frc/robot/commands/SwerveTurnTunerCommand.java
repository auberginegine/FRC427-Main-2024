package frc.robot.commands;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;

// command to tune the turn PID a single swerve module
public class SwerveTurnTunerCommand extends Command {
    SwerveModule module; 

    /***
     * 
     * @param turn the CAN ID of the turn motor
     * @param drive the CAN ID of the drive motor
     * @param encoder the CAN ID of the turn encoder
     */
    public SwerveTurnTunerCommand(SwerveModuleConfig config) {
        this.module = new SwerveModule(config); 
    }

    @Override
    public void initialize() {
        // initialize the turn PID values of the module to the dashboard
        SmartDashboard.putNumber("SwervekP", SmartDashboard.getNumber("SwervekP", 0));
        SmartDashboard.putNumber("SwervekI", SmartDashboard.getNumber("SwervekI", 0));
        SmartDashboard.putNumber("SwervekD", SmartDashboard.getNumber("SwervekD", 0)); 
        SmartDashboard.putNumber("SwerveTurnSetpoint", SmartDashboard.getNumber("SwerveTurnSetpoint", 0)); 
    }

    @Override
    public void execute() {
        // set PID values periodically
        module.setTurnPID(
            SmartDashboard.getNumber("SwervekP", 0), 
            SmartDashboard.getNumber("SwervekI", 0), 
            SmartDashboard.getNumber("SwervekD", 0)
            );

        // drive the module with the desired angle setpoint (from dashboard)
        module.updateState(new SwerveModuleState(0, Rotation2d.fromDegrees(
            SmartDashboard.getNumber("SwerveTurnSetpoint", 0))
        ), SwerveModule.DriveState.OPEN_LOOP);

        SmartDashboard.putNumber("SwerveTurnMeasurement", module.getAngle().getDegrees()); 
        SmartDashboard.putNumber("SwerveError", module.turnPIDController.getPositionError()); 
    }

    @Override
    public boolean isFinished() {
        // never finishes until cancelled
        return false; 
    }
}
