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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.automations.Shoot;
import frc.robot.subsystems.intake.Elevator;;

public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;
    private Shoot m_Shoot;
    private Elevator m_Elevator;

    // Left and Right are on the center
    // Front and Back are close to swerve encoders

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private PhotonCamera camLeft;
    private PhotonCamera camRight;
    private PhotonCamera camShooter;
    private PhotonCamera camIntake;

    private PhotonPoseEstimator visionLeft;
    private PhotonPoseEstimator visionRight;
    private PhotonPoseEstimator visionShooter;
    private PhotonPoseEstimator visionIntake;

    public Optional<EstimatedRobotPose> poseLeft;
    public Optional<EstimatedRobotPose> poseRight;
    public Optional<EstimatedRobotPose> poseShooter;
    public Optional<EstimatedRobotPose> poseIntake;

    public Translation2d lastLeftTranslation;
    public Translation2d lastRightTranslation;

    public double lastLeftTime;
    public double lastRightTime;

    public Field2d m_field = new Field2d();

    public Vision(SwerveDrive m_SwerveDrive, Shoot m_shoot, Elevator m_Elevator) {
        // camLeft = new PhotonCamera("camLeft");
        // camRight = new PhotonCamera("camRight");
        camShooter = new PhotonCamera("camShooter");
        camIntake = new PhotonCamera("camIntake");

        // visionLeft = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft, new Transform3d(new
        // Translation3d(0, -0.32385, 0.5969), new Rotation3d(0,-Math.PI/4,Math.PI/2)));
        // visionRight = new PhotonPoseEstimator(aprilTagFieldLayout,
        // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRight, new Transform3d(new
        // Translation3d(0, 0.32385, 0.5969), new Rotation3d(0,-Math.PI/4,
        // -Math.PI/2)));
        visionShooter = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camShooter,
                new Transform3d(new Translation3d(0, 0, 0.66),
                        new Rotation3d(Units.degreesToRadians(-2.7), 0, Math.PI)));
        visionIntake = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camIntake,
                new Transform3d(new Translation3d(.152, 0, 0.66), new Rotation3d(0, -Units.degreesToRadians(40), 0)));

        this.m_SwerveDrive = m_SwerveDrive;
        this.m_Shoot = m_shoot;
        this.m_Elevator = m_Elevator;

    }

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator photonPoseEstimator,
            Pose2d prevEstimatedPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update();
    }

    public double getDistanceFromTarget(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }

    public Boolean isTargetCloseEnough(Optional<EstimatedRobotPose> estimatedRobotPose) {

        for (PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
            if (getDistanceFromTarget(target) > 4.5) {
                return false;
            }
        }

        return true;
    }

    public void addVisionMeasurement() {
        // poseLeft = getEstimatedPose(visionLeft, m_SwerveDrive.getPose());
        // poseRight = getEstimatedPose(visionRight, m_SwerveDrive.getPose());
        poseShooter = getEstimatedPose(visionShooter, m_SwerveDrive.getPose());
        poseIntake = getEstimatedPose(visionIntake, m_SwerveDrive.getPose());

        if (poseShooter.isPresent() && isTargetCloseEnough(poseShooter)) {
            if (poseShooter.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength()
                    && poseShooter.get().estimatedPose.getX() > 0
                    && poseShooter.get().estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth()
                    && poseShooter.get().estimatedPose.getY() > 0)
                m_SwerveDrive.m_odometry.addVisionMeasurement(poseShooter.get().estimatedPose.toPose2d(),
                        poseShooter.get().timestampSeconds);
            m_field.getObject("poseShooter").setPose(poseShooter.get().estimatedPose.toPose2d());

        }

        // if (poseLeft.isPresent() && isTargetCloseEnough(poseLeft)) {
        // m_SwerveDrive.m_odometry.addVisionMeasurement(poseLeft.get().estimatedPose.toPose2d(),
        // poseLeft.get().timestampSeconds);
        // m_field.getObject("poseLeft").setPose(poseLeft.get().estimatedPose.toPose2d());

        // }

        // if (poseRight.isPresent() && isTargetCloseEnough(poseRight)) {
        // m_SwerveDrive.m_odometry.addVisionMeasurement(poseRight.get().estimatedPose.toPose2d(),
        // poseRight.get().timestampSeconds);
        // m_field.getObject("poseRight").setPose(poseRight.get().estimatedPose.toPose2d());
        // }

        // if (poseIntake.isPresent() && isTargetCloseEnough(poseIntake) && isIntakeVisionViable()
        //         && poseIntake.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength()
        //         && poseIntake.get().estimatedPose.getX() > 0
        //         && poseIntake.get().estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth()
        //         && poseIntake.get().estimatedPose.getY() > 0) {
        //     m_SwerveDrive.m_odometry.addVisionMeasurement(poseIntake.get().estimatedPose.toPose2d(),
        //             poseIntake.get().timestampSeconds);
        //     m_field.getObject("poseIntake").setPose(poseIntake.get().estimatedPose.toPose2d());
        // }

        m_field.setRobotPose(m_SwerveDrive.getPose());

    }

    public boolean isIntakeVisionViable() {
        if (m_Elevator.getElevPos() > 14) {
            return false;
        }
        return true;
    }

    public void updateSmartDashBoard() {
        if (poseLeft != null && poseLeft.isPresent()) {
            SmartDashboard.putNumber("camleft x", poseLeft.get().estimatedPose.getX());
            SmartDashboard.putNumber("camleft y", poseLeft.get().estimatedPose.getY());
        }

        if (poseRight != null && poseRight.isPresent()) {
            SmartDashboard.putNumber("camright x", poseRight.get().estimatedPose.getX());
            SmartDashboard.putNumber("camright y", poseRight.get().estimatedPose.getY());
        }

        if (poseShooter != null && poseShooter.isPresent()) {
            SmartDashboard.putNumber("camShooter x", poseShooter.get().estimatedPose.getX());
            SmartDashboard.putNumber("camShooter y", poseShooter.get().estimatedPose.getY());
        }

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {
  
        addVisionMeasurement();

    }
}