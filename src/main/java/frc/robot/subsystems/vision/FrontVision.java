package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

// Figures out which game pieces are near
public class FrontVision extends SubsystemBase{
     public static FrontVision instance; 
     //  = new FrontVision(Drivetrain.getInstance());

    private Drivetrain drivetrain;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult;
    private Transform3d latestPoseResult;

    private FrontVision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.camera = new PhotonCamera("frontPhotonCamera");
    }

    public void periodic() {
        this.latestResult = this.camera.getLatestResult();
        this.latestPoseResult = this.latestResult.getMultiTagResult().estimatedPose.best;
        SmartDashboard.putNumber("Target Yaw",getNoteRotation());

    }

    public static FrontVision getInstance() {
        return instance;
    }

    public PhotonPipelineResult getLatestVisionResult() {
        return latestResult;
    }

    public double getNoteRotation() {
        return latestResult.getBestTarget().getYaw();
    }

}
