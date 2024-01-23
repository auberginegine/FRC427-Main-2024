package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase{

private static NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");

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
double limelightX = botPoseValues[0];
double limelightY = botPoseValues[1];
double limelightZ = botPoseValues[2];
double limelightRoll = botPoseValues[3];
double limelightPitch = botPoseValues[4];
double limelightYaw = botPoseValues[5];
double limelightTotalLatency = botPoseValues[6];
double limelightTargetArea = limelightNT.getEntry("<tA>").getDouble(0);
double limelightTargetX = limelightNT.getEntry("<tX>").getDouble(0);
double limelightTargetY = limelightNT.getEntry("<tY>").getDouble(0);
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
