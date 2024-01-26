package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Limelight extends SubsystemBase{

private static NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
private static double limelightX;
private static double limelightY;
private static double limelightZ;
private static double limelightRoll;
private static double limelightPitch;
private static double limelightYaw;
private static double limelightTotalLatency;
private static double limelightTargetArea;
private static double limelightTargetX;
double limelightTargetY;

public Limelight() {
    limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
}

public void periodic() {
    // Gets the values scored in the table
NetworkTableEntry botPose = limelightNT.getEntry("botPose");
NetworkTableEntry camPose = limelightNT.getEntry("camPose");

    // Reads values periodically
double[] botPoseValues = botPose.getDoubleArray(new double[6]); // Make sure these are the right indices
    // Converts these values into variables
limelightX = botPoseValues[0];
limelightY = botPoseValues[1];
limelightZ = botPoseValues[2];
limelightRoll = botPoseValues[3];
limelightPitch = botPoseValues[4];
limelightYaw = botPoseValues[5];
limelightTotalLatency = botPoseValues[6];
limelightTargetArea = limelightNT.getEntry("<tA>").getDouble(0);
limelightTargetX = limelightNT.getEntry("<tX>").getDouble(0);
limelightTargetY = limelightNT.getEntry("<tY>").getDouble(0);

    // Posts to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", limelightX);
SmartDashboard.putNumber("LimelightY", limelightY);
SmartDashboard.putNumber("LimelightZ", limelightZ);
SmartDashboard.putNumber("LimelightRoll", limelightRoll);
SmartDashboard.putNumber("LimelightPitch", limelightPitch);
SmartDashboard.putNumber("LimelightYaw", limelightYaw);
SmartDashboard.putNumber("LimelightTotalLatency", limelightTotalLatency);
SmartDashboard.putNumber("LimelightTargetArea", limelightTargetArea);
SmartDashboard.putNumber("LimelightTargetX", limelightTargetX);
SmartDashboard.putNumber("LimelightTargetY", limelightTargetY);
}

}