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

    public double getDistanceToTarget(PhotonTrackedTarget target, PhotonTrackedTarget bestTarget) {
        return target.getBestCameraToTarget().getTranslation()
                .minus(bestTarget.getBestCameraToTarget().getTranslation()).getNorm();
    }

    public EstimatedPoseInfo getEstimatedPoseInfo(PhotonCamera camera) {
        Pose3d cameraOffset;
        int i = 0;
        PhotonCamera cam = camera;
        PhotonPipelineResult result = cam.getLatestResult();
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        ArrayList<Pose2d> filteredResults;
        ArrayList<Double> filteredResultsTime;
        Pose2d totalEstimatedPose2d;
        Pose2d averageEstimatedPose2d;
        filteredResults = new ArrayList<Pose2d>();
        filteredResultsTime = new ArrayList<Double>();
        totalEstimatedPose2d = new Pose2d();
        averageEstimatedPose2d = new Pose2d();

        if (cam.getName() == "camShooter") {
            cameraOffset = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        }

        if (cam.getName() == "camIntake") {
            cameraOffset = new Pose3d(new Translation3d(.152, 0, getIntakeVisionOffset()),
                    new Rotation3d(0, -Math.PI / 4, 0));

        } else {
            cameraOffset = new Pose3d();
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            boolean rotatingTooFast = Math.abs(m_SwerveDrive.currentMovement.omegaRadiansPerSecond) >= 1.0;

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
            if ((getDistanceToTarget(target, bestTarget) < 0.1)) {
                continue;
            }

            filteredResults.add(
                    getTargetToRobot(target, cameraOffset, m_SwerveDrive.getPose()));
        }

        for (i = 0; i < filteredResults.size(); i++) {
            totalEstimatedPose2d = new Pose2d(totalEstimatedPose2d.getX() + filteredResults.get(i).getX(),
                    totalEstimatedPose2d.getY() + filteredResults.get(i).getY(), filteredResults.get(0).getRotation());
        }

        averageEstimatedPose2d = totalEstimatedPose2d.div(i);

        return new EstimatedPoseInfo(averageEstimatedPose2d, result.getTimestampSeconds());
    }

    public Pose2d getTargetToRobot(PhotonTrackedTarget target, Pose3d offset, Pose2d robotPose2d) {
        Transform3d tagPose;
        Pose2d targetToRobot;
        Pose2d robotToField;

        double x;
        double y;
        double rotation;

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().minus(offset);
        } else {
            tagPose = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        }

        x = tagPose.getZ() / Math.tan(target.getPitch());
        y = Math.tan(target.getYaw()) * x;
        rotation = tagPose.getRotation().toRotation2d().getDegrees() - (target.getYaw());

        targetToRobot = new Pose2d(x, y, new Rotation2d().fromDegrees(rotation));

        robotToField = new Pose2d((tagPose.getTranslation().toTranslation2d().plus(robotPose2d.getTranslation())),
                targetToRobot.getRotation());

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