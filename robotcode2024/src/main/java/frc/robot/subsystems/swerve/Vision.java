package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.*;

public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;

    // Left and Right are on the center
    // Front and Back are close to swerve encoders
    private PhotonCamera camLeft;
    private PhotonCamera camRight;
    private PhotonCamera camFront;
    private PhotonCamera camBack;

    public CameraInfo visionLeft;
    public CameraInfo visionRight;
    public CameraInfo visionFront;
    public CameraInfo visionBack;

    public Vision(SwerveDrive m_SwerveDrive) {
        camLeft = new PhotonCamera("camLeft");
        camRight = new PhotonCamera("camRight");
        camFront = new PhotonCamera("camFront");
        camBack = new PhotonCamera("camBack");

        this.m_SwerveDrive = m_SwerveDrive;

        visionLeft = new CameraInfo(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0, false);
        visionRight = new CameraInfo(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0, false);
        ;
        visionFront = new CameraInfo(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0, false);
        ;
        visionBack = new CameraInfo(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), 0, false);
        ;

    }

    public class CameraInfo {
        private Pose2d pose2d;
        private double timeStamp;
        private boolean isPresent;

        CameraInfo(Pose2d pose2d, double timeStamp, boolean isPresent) {
            this.pose2d = pose2d;
            this.timeStamp = timeStamp;
            this.isPresent = isPresent;
        }

        public Pose2d getPose2d() {
            return pose2d;
        }

        public double getTimeStamp() {
            return timeStamp;
        }

        public boolean hasTag() {
            return isPresent;
        }
    }

    public CameraInfo getVisionInfo(PhotonCamera camera) {
        var result = camera.getLatestResult();

        Transform3d fieldToCamera;
        Pose2d fieldToCameraPose2d;
        double cameraTimeStamp;
        boolean hasTag;

        hasTag = result.getMultiTagResult().estimatedPose.isPresent;

        fieldToCamera = result.getMultiTagResult().estimatedPose.best;

        fieldToCameraPose2d = new Pose2d(new Translation2d(fieldToCamera.getX(), fieldToCamera.getY()),
                new Rotation2d(fieldToCamera.getRotation().getZ()));
        cameraTimeStamp = result.getTimestampSeconds();

        return new CameraInfo(fieldToCameraPose2d, cameraTimeStamp, hasTag);
    }

    public void addVisionMeasurement() {
        visionLeft = getVisionInfo(camLeft);
        visionRight = getVisionInfo(camRight);
        visionFront = getVisionInfo(camFront);
        visionBack = getVisionInfo(camBack);

        if (visionLeft.hasTag()) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(visionLeft.getPose2d(), visionLeft.getTimeStamp());
        }
        if (visionRight.hasTag()) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(visionRight.getPose2d(), visionRight.getTimeStamp());
        }
        if (visionFront.hasTag()) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(visionFront.getPose2d(), visionFront.getTimeStamp());
        }
        if (visionBack.hasTag()) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(visionBack.getPose2d(), visionBack.getTimeStamp());
        }
    }

    public void updateSmartDashBoard() {

    }

    public void periodic() {

        addVisionMeasurement();

    }
}
