package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;

    // Left and Right are on the center
    // Front and Back are close to swerve encoders

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private PhotonCamera camLeft;
    private PhotonCamera camRight;
    private PhotonCamera camFront;
    private PhotonCamera camBack;

    private PhotonPoseEstimator visionLeft;
    private PhotonPoseEstimator visionRight;
    private PhotonPoseEstimator visionFront;
    private PhotonPoseEstimator visionBack;

    public Optional<EstimatedRobotPose> poseLeft;
    public Optional<EstimatedRobotPose> poseRight;
    public Optional<EstimatedRobotPose> poseFront;
    public Optional<EstimatedRobotPose> poseBack;

    public Vision(SwerveDrive m_SwerveDrive) {
        camLeft = new PhotonCamera("camLeft");
        camRight = new PhotonCamera("camRight");
        camFront = new PhotonCamera("camFront");
        camBack = new PhotonCamera("camBack");

        visionLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft,
                new Transform3d(new Translation3d(0, -0.32385, 0.5969),
                        new Rotation3d(Math.PI, -Math.PI / 4, Math.PI / 2)));

        this.m_SwerveDrive = m_SwerveDrive;

    }

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator photonPoseEstimator,
            Pose2d prevEstimatedPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update();
    }

    public void addVisionMeasurement() {
        poseLeft = getEstimatedPose(visionLeft, m_SwerveDrive.getPose());

        if (poseLeft.isPresent() && poseLeft.get().estimatedPose.getX() > 0
                && poseLeft.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength()) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(poseLeft.get().estimatedPose.toPose2d(),
                    poseLeft.get().timestampSeconds);
        }
    }

    public void updateSmartDashBoard() {
        if (poseLeft != null && poseLeft.isPresent()) {
            SmartDashboard.putNumber("camleft x", poseLeft.get().estimatedPose.getX());
            SmartDashboard.putNumber("camleft y", poseLeft.get().estimatedPose.getY());
        }
    }

    public void periodic() {

        addVisionMeasurement();

    }
}
