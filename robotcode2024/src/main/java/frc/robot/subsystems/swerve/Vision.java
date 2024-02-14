package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;

    // Left and Right are on the center
    // Front and Back are close to swerve encoders
    private PhotonCamera camLeft;
    private PhotonCamera camRight;
    private PhotonCamera camFront;
    private PhotonCamera camBack;

    public Pose2d visionPoseLeftPose2d;
    public Pose2d visionPoseRightPose2d;
    public Pose2d visionPoseFrontPose2d;
    public Pose2d visionPoseBackPose2d;

    public boolean leftHasTarget;
    public boolean rightHasTarget;
    public boolean frontHasTarget;
    public boolean backHasTarget;

    public Vision(SwerveDrive m_SwerveDrive) {
        camLeft = new PhotonCamera("camLeft");
        camRight = new PhotonCamera("camRight");
        camFront = new PhotonCamera("camFront");
        camBack = new PhotonCamera("camBack");

        this.m_SwerveDrive = m_SwerveDrive;

    }

    public class CameraInfo {
        private Pose2d pose2d;
        private double latency;
        private PhotonPipelineResult result;

        CameraInfo(Pose2d pose2d, double latency, PhotonPipelineResult) {
            this.pose2d = pose2d;
            this.latency = latency;
            this.result = result;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }

        public double getLatency() {
            return latency;
        }
    }

    public CameraInfo getVisionPose(PhotonCamera camera) {
        var result = camera.getLatestResult();

        Transform3d fieldToCamera;
        Pose2d fieldToCameraPose2d;
        double cameraLatency;
        if (result.getMultiTagResult().estimatedPose.isPresent) {
            fieldToCamera = result.getMultiTagResult().estimatedPose.best;
        }
        fieldToCameraPose2d = new Pose2d(new Translation2d(fieldToCamera.getX(), fieldToCamera.getY()),
                new Rotation2d(fieldToCamera.getRotation().getAngle()));
        cameraLatency = result.getLatencyMillis();

        return new CameraInfo(fieldToCameraPose2d, cameraLatency);
    }

    public void updateSmartDashBoard() {

    }

    public void periodic() {

    }
}
