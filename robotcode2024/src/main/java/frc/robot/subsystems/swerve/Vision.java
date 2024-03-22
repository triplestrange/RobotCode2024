package frc.robot.subsystems.swerve;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.commands.automations.Shoot;
import frc.robot.subsystems.intake.Elevator;;

public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;
    private Shoot m_Shoot;
    private Elevator m_Elevator;

    // Left and Right are on the center
    // Front and Back are close to swerve encoders

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private PhotonCamera camShooter;
    private PhotonCamera camIntake;

    public EstimatedPoseInfo poseShooter;
    public EstimatedPoseInfo poseIntake;

    public Translation2d lastLeftTranslation;
    public Translation2d lastRightTranslation;

    public double lastLeftTime;
    public double lastRightTime;

    public Field2d m_field = new Field2d();

    public Vision(SwerveDrive m_SwerveDrive, Shoot m_shoot, Elevator m_Elevator) {
        camShooter = new PhotonCamera("camShooter");
        camIntake = new PhotonCamera("camIntake");

        this.m_SwerveDrive = m_SwerveDrive;
        this.m_Shoot = m_shoot;
        this.m_Elevator = m_Elevator;

    }

    public EstimatedPoseInfo getEstimatedPoseInfo(PhotonCamera camera) {
        Pose3d cameraOffset;
        int i = 0;
        PhotonCamera cam = camera;
        PhotonPipelineResult result = cam.getLatestResult();
        ArrayList<Pose2d> filteredResults;
        Translation2d totalEstimatedTranslation2d;
        Pose2d averageEstimatedPose2d;
        filteredResults = new ArrayList<Pose2d>();
        totalEstimatedTranslation2d = new Translation2d();
        averageEstimatedPose2d = new Pose2d();

        if (cam.getName() == "camShooter") {
            cameraOffset = new Pose3d(new Translation3d(0, 0, 0.66),
                    new Rotation3d(Units.degreesToRadians(-2.7), 0, Math.PI));
        }

        if (cam.getName() == "camIntake") {
            cameraOffset = new Pose3d(new Translation3d(.152, 0, 0.66), new Rotation3d(0, -Math.PI / 4, 0));

        } else {
            cameraOffset = new Pose3d();
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            boolean rotatingTooFast = Math.abs(m_SwerveDrive.currentMovement.omegaRadiansPerSecond) >= Math.PI;

            if (target.getBestCameraToTarget().getTranslation().getNorm() > 4.5) {
                continue;
            }
            if (rotatingTooFast) {
                continue;
            }

            if (!(target.getBestCameraToTarget().getTranslation().getX() < aprilTagFieldLayout.getFieldLength())
                    || !(target.getBestCameraToTarget().getTranslation().getX() > 0)
                    || !(target.getBestCameraToTarget().getTranslation().getY() < aprilTagFieldLayout.getFieldWidth())
                    || !(target.getBestCameraToTarget().getTranslation().getY() > 0)) {
                continue;
            }

            if (target.getArea() > 50) {
                continue;
            }

            filteredResults.add(
                    getTargetToRobot(target, cameraOffset, m_SwerveDrive.getPose()));
        }

        for (i = 0; i < filteredResults.size(); i++) {
            totalEstimatedTranslation2d = new Translation2d(
                    totalEstimatedTranslation2d.getX() + filteredResults.get(i).getX(),
                    totalEstimatedTranslation2d.getY() + filteredResults.get(i).getY());
        }

        averageEstimatedPose2d = new Pose2d(totalEstimatedTranslation2d.div(i), m_SwerveDrive.getPose().getRotation());

        return new EstimatedPoseInfo(averageEstimatedPose2d, result.getTimestampSeconds());
    }

    public Pose2d getTargetToRobot(PhotonTrackedTarget target, Pose3d offset, Pose2d robotPose2d) {
        Pose3d tagPose;
        Translation2d cameraToTarget;
        Translation2d robotToTarget;
        Pose2d robotToField;
        Translation3d xyz_plane_translation;

        double x;
        double y;
        double z;
        Rotation2d rotation;

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
        } else {
            tagPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        }

        xyz_plane_translation = new Translation3d(1, Math.tan(target.getYaw()),
                Math.tan(target.getPitch()))
                .rotateBy(
                        offset.getRotation());

        x = xyz_plane_translation.getX();
        y = xyz_plane_translation.getY();
        z = xyz_plane_translation.getZ();
        rotation = robotPose2d.getRotation();

        cameraToTarget = new Translation2d(x, y).times((tagPose.getZ() - offset.getZ()) / z);

        robotToTarget = new Translation2d(cameraToTarget.getX() - offset.getX(), cameraToTarget.getY() - offset.getY())
                .rotateBy(rotation);

        robotToField = new Pose2d((tagPose.getTranslation().toTranslation2d().plus(robotPose2d.getTranslation())),
                rotation);

        return robotToField;
    }

    public void addVisionMeasurement() {
        poseShooter = getEstimatedPoseInfo(camShooter);
        poseIntake = getEstimatedPoseInfo(camIntake);

        m_SwerveDrive.m_odometry.addVisionMeasurement(poseShooter.getPose2d(),
                poseShooter.getTimestampSeconds());
        m_field.getObject("poseShooter").setPose(poseShooter.getPose2d());

        m_SwerveDrive.m_odometry.addVisionMeasurement(poseIntake.getPose2d(),
                poseIntake.getTimestampSeconds());
        m_field.getObject("poseIntake").setPose(poseIntake.getPose2d());

        m_field.setRobotPose(m_SwerveDrive.getPose());

    }

    public double getIntakeVisionOffset() {
        if (m_Elevator.getElevPos() > 14) {
            return 0.66 + Units.inchesToMeters(m_Elevator.getElevPos() - 14);
        }
        return 0.66;
    }

    public void updateSmartDashBoard() {

        if (poseShooter.getPose2d() != null) {
            SmartDashboard.putNumber("camShooter x", poseShooter.getPose2d().getX());
            SmartDashboard.putNumber("camShooter y", poseShooter.getPose2d().getY());
        }

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {

        addVisionMeasurement();

    }

    public static class EstimatedPoseInfo {
        private Pose2d pos;
        private double timestampSeconds;

        public EstimatedPoseInfo(Pose2d pos, double timestampSeconds) {
            this.pos = pos;
            this.timestampSeconds = timestampSeconds;
        }

        public Pose2d getPose2d() {
            return pos;
        }

        public double getTimestampSeconds() {
            return timestampSeconds;
        }
    }
}