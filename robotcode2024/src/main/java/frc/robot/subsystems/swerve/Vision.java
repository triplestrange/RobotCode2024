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

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
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

    private PhotonCamera camShooter;
    private PhotonCamera camIntake;

    private PhotonPipelineResult visionShooter;
    private PhotonPipelineResult visionIntake;

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
        camShooter = new PhotonCamera("camShooter");
        camIntake = new PhotonCamera("camIntake");

        visionShooter = camShooter.getLatestResult();
        visionIntake = camIntake.getLatestResult();

        this.m_SwerveDrive = m_SwerveDrive;
        this.m_Shoot = m_shoot;
        this.m_Elevator = m_Elevator;

    }

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonCamera camera,
            Pose2d prevEstimatedPose) {

        return photonPoseEstimator.update();
    }

    public double getDistanceToTarget(PhotonTrackedTarget target, PhotonTrackedTarget bestTarget) {
        return target.getBestCameraToTarget().getTranslation()
                .minus(bestTarget.getBestCameraToTarget().getTranslation()).getNorm();
    }

    public double getTargetPose2d(PhotonCamera camera) {
        double[] cameraOffset = new double[5];

        
        PhotonCamera cam = camera;
        PhotonPipelineResult result = cam.getLatestResult();
        ArrayList<Pose2d> filteredResults;

        if (cam.getName() == "camShooter")  {
            cameraOffset[0] = 0;
            cameraOffset[1] = 0;
            cameraOffset[2] = 0;
            cameraOffset[3] = 0;
            cameraOffset[4] = 0;
            cameraOffset[5] = 0;

        }   

        if (cam.getName() == "camIntake")   {
            cameraOffset[0] = 0;
            cameraOffset[1] = 0;
            cameraOffset[2] = 0;
            cameraOffset[3] = 0;
            cameraOffset[4] = 0;
            cameraOffset[5] = 0;        
        }

        for (PhotonTrackedTarget target : result.getTargets()) {
            if (!(getTargetPose2d(cam) > 0.1)) {
                filteredResults.add(
                    new Pose2d(
                        new Translation2d(
                            target.getYaw()
                        )
                    )

                ) 
            }
        }
    }

    public Boolean isTargetCloseEnough(Optional<EstimatedRobotPose> estimatedRobotPose) {

        for (PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
            if (target.getBestCameraToTarget().getTranslation().getNorm() > 4.5) {
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

        if (poseIntake.isPresent() && isTargetCloseEnough(poseIntake)
                && poseIntake.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength()
                && poseIntake.get().estimatedPose.getX() > 0
                && poseIntake.get().estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth()
                && poseIntake.get().estimatedPose.getY() > 0) {
            m_SwerveDrive.m_odometry.addVisionMeasurement(poseIntake.get().estimatedPose.toPose2d(),
                    poseIntake.get().timestampSeconds);
            m_field.getObject("poseIntake").setPose(poseIntake.get().estimatedPose.toPose2d());
        }

        m_field.setRobotPose(m_SwerveDrive.getPose());

    }

    public double getIntakeVisionOffset() {
        if (m_Elevator.getElevPos() > 14) {
            return 0.66 + Units.inchesToMeters(m_Elevator.getElevPos() - 14);
        }
        return 0.66;
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
        if (camIntake != null) {
            visionIntake.setRobotToCameraTransform(new Transform3d(new Translation3d(.152, 0, getIntakeVisionOffset()),
                    new Rotation3d(0, -Math.PI / 4, 0)));
        }
        addVisionMeasurement();

    }
}