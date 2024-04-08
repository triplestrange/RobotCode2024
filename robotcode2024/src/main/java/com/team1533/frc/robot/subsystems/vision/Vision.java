package com.team1533.frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.photonvision.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.team1533.frc.robot.RobotContainer;
import static org.opencv.core.CvType.CV_64FC1;

public class Vision extends SubsystemBase {

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private RobotContainer m_RobotContainer;

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

    public PhotonTrackedTarget testTarget;

    private Mat mCameraMatrix = new Mat(3, 3, CV_64FC1);
    private Mat mDistortionCoeffients = new Mat(1, 5, CV_64FC1);

    public Field2d m_field = new Field2d();
    @AutoLogOutput
    private Pose2d poseShooterActual;
    @AutoLogOutput
    private Pose2d poseIntakeActual;

    private static Comparator<Translation2d> ySort;

    public Vision(RobotContainer m_RobotContainer) {
        camShooter = new PhotonCamera("camShooter");
        camIntake = new PhotonCamera("camIntake");

        this.m_RobotContainer = m_RobotContainer;

        ySort = Comparator.comparingDouble(Translation2d::getY);

    }

    @AutoLogOutput

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

        for (PhotonTrackedTarget target : result.getTargets()) {
            boolean rotatingTooFast = Math
                    .abs(m_RobotContainer.m_robotDrive.currentMovement.omegaRadiansPerSecond) >= Math.PI;

            if (rotatingTooFast) {
                continue;
            }

            if (cam.getCameraMatrix().isPresent() && cam.getDistCoeffs().isPresent()) {
                filteredResults.add(
                        getRobotToField(target, cam, m_RobotContainer.m_robotDrive.getPose()));
            }
        }

        for (i = 0; i < filteredResults.size(); i++) {
            totalEstimatedTranslation2d = new Translation2d(
                    totalEstimatedTranslation2d.getX() + filteredResults.get(i).getX(),
                    totalEstimatedTranslation2d.getY() + filteredResults.get(i).getY());
        }

        averageEstimatedPose2d = new Pose2d(totalEstimatedTranslation2d.div(i),
                m_RobotContainer.m_robotDrive.getPose().getRotation());

        return new EstimatedPoseInfo(averageEstimatedPose2d, result.getTimestampSeconds(), filteredResults.size());
    }

    @AutoLogOutput

    public Pose2d getBestObject(Pose2d robotPose) {
        PhotonCamera cam = camIntake;
        PhotonTrackedTarget target = cam.getLatestResult().getBestTarget();

        if (target == null) {
            return null;
        }

        return getObjectToField(getObjectToRobot(target, cam, robotPose));
    }

    @AutoLogOutput

    public Pose2d getRobotToField(PhotonTrackedTarget target, PhotonCamera cam, Pose2d robotPose2d) {
        Pose3d tagPose;
        Translation2d cameraToTarget;
        ArrayList<Translation2d> robotToPoints = new ArrayList<Translation2d>();
        Translation2d robotToTarget = new Translation2d();
        Pose2d robotToField;
        Translation3d xyz_plane_translation;
        Pose3d cameraOffset;

        List<Translation2d> desiredTargetPixel = new ArrayList<Translation2d>();

        double x;
        double y;
        double z;
        Rotation2d rotation;

        rotation = robotPose2d.getRotation();

        if (cam.getName().equals("camShooter")) {
            cameraOffset = new Pose3d(new Translation3d(0, 0, 0.66),
                    new Rotation3d(Units.degreesToRadians(-2.7), 0, Math.PI));
        }

        else if (cam.getName().equals("camIntake")) {
            cameraOffset = new Pose3d(new Translation3d(.152, 0, getIntakeVisionOffset()),
                    new Rotation3d(Math.PI, -Units.degreesToRadians(40), 0));

        } else {
            cameraOffset = new Pose3d();
        }

        for (TargetCorner corner : target.getDetectedCorners()) {

            desiredTargetPixel.add(undistortFromOpenCV((new Translation2d(corner.x, corner.y)), cam));
        }

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
        } else {
            tagPose = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
        }
        int i = 0;

        for (Translation2d point : desiredTargetPixel) {

            xyz_plane_translation = new Translation3d(1, point.getX(),
                    point.getY())
                    .rotateBy(
                            cameraOffset.getRotation());

            x = xyz_plane_translation.getX();
            y = xyz_plane_translation.getY();
            z = xyz_plane_translation.getZ();

            double offset = i == 2 || i == 3 ? Units.inchesToMeters(3.25) : -Units.inchesToMeters(3.25);

            cameraToTarget = new Translation2d(x, y).times((tagPose.getZ() - cameraOffset.getZ() + offset) / z);

            robotToPoints.add(new Translation2d(cameraToTarget.getX() + cameraOffset.getX(),
                    cameraToTarget.getY() + cameraOffset.getY())
                    .rotateBy(rotation));

            i++;

        }

        for (Translation2d translation : robotToPoints) {
            robotToTarget = robotToTarget.plus(translation);
        }

        robotToField = new Pose2d((tagPose.getTranslation().toTranslation2d().minus(robotToTarget.div(4))),
                rotation);

        return robotToField;
    }

    @AutoLogOutput

    public Pose2d getObjectToRobot(PhotonTrackedTarget target, PhotonCamera cam, Pose2d robotPose2d) {
        Translation2d cameraToTarget;
        ArrayList<Translation2d> robotToPoints = new ArrayList<Translation2d>();
        Translation2d robotToTarget = new Translation2d();
        Pose2d robotToTargetPose2d = new Pose2d();
        Translation3d xyz_plane_translation;
        Pose3d cameraOffset;

        List<Translation2d> desiredTargetPixel = new ArrayList<Translation2d>();

        double x;
        double y;
        double z;
        Rotation2d rotation;

        rotation = robotPose2d.getRotation();

        if (cam.getName().equals("camShooter")) {
            cameraOffset = new Pose3d(new Translation3d(0, 0, 0.66),
                    new Rotation3d(Units.degreesToRadians(-2.7), 0, Math.PI));
        }

        else if (cam.getName().equals("camIntake")) {
            cameraOffset = new Pose3d(new Translation3d(Units.inchesToMeters(5.75), 0, getIntakeVisionOffset()),
                    new Rotation3d(Math.PI / 2.0, -Units.degreesToRadians(2), 0));

        } else {
            cameraOffset = new Pose3d();
        }

        for (TargetCorner corner : target.getDetectedCorners()) {

            desiredTargetPixel.add(undistortFromOpenCV((new Translation2d(corner.x, corner.y)), cam));
        }

        desiredTargetPixel.sort(ySort);

        double yOffset = (desiredTargetPixel.get(2).getY() - desiredTargetPixel.get(0).getY()) / 2;

        int i = 0;

        for (Translation2d point : desiredTargetPixel) {

            xyz_plane_translation = new Translation3d(1, point.getX(),
                    point.getY())
                    .rotateBy(
                            cameraOffset.getRotation());

            x = xyz_plane_translation.getX();
            y = xyz_plane_translation.getY();
            z = xyz_plane_translation.getZ();

            double offset = i == 2 || i == 3 ? yOffset : -yOffset;

            cameraToTarget = new Translation2d(x, y).times((0 - cameraOffset.getZ() + offset) / z);

            robotToPoints.add(new Translation2d(cameraToTarget.getX() + cameraOffset.getX(),
                    cameraToTarget.getY() + cameraOffset.getY())
                    .rotateBy(rotation));

            i++;

        }

        for (Translation2d translation : robotToPoints) {
            robotToTarget = robotToTarget.plus(translation);
        }

        robotToTargetPose2d = new Pose2d(((robotToTarget.div(4))),
                rotation);

        return robotToTargetPose2d;

    }

    @AutoLogOutput

    public Pose2d getObjectToField(Pose2d objectToRobot) {
        return new Pose2d(
                objectToRobot.getTranslation()
                        .minus(m_RobotContainer.m_robotDrive.getPose().getTranslation()),
                m_RobotContainer.m_robotDrive.getPose().getRotation());
    }

    public Translation2d undistortFromOpenCV(Translation2d point, PhotonCamera cam) {
        Point coord = new Point();
        coord.x = point.getX();
        coord.y = point.getY();

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                mCameraMatrix.put(i, j, cam.getCameraMatrix().get().get(i, j));
            }
        }
        for (int i = 0; i < 5; i++) {
            mDistortionCoeffients.put(0, i, cam.getDistCoeffs().get().get(i, 0));
        }

        MatOfPoint2f coordMat = new MatOfPoint2f(coord);

        Point dstCoord = new Point();
        MatOfPoint2f dst = new MatOfPoint2f(dstCoord);

        Calib3d.undistortImagePoints(coordMat, dst, mCameraMatrix, mDistortionCoeffients);

        double x_pixels = dst.get(0, 0)[0];
        double y_pixels = dst.get(0, 0)[1];

        // Negate OpenCV Undistorted Pixel Values to Match Robot Frame of Reference
        // OpenCV: Positive Downward and Right
        // Robot: Positive Upward and Left
        double nX = -(x_pixels - mCameraMatrix.get(0, 2)[0]);// -(y_pixels * 2.0 - 1.0);
        double nY = -(y_pixels - mCameraMatrix.get(1, 2)[0]);// -(z_pixels * 2.0 - 1.0);

        double x = nX / mCameraMatrix.get(0, 0)[0];
        double y = nY / mCameraMatrix.get(1, 1)[0];

        return new Translation2d(x, y);
    }

    public void addVisionMeasurement() {
        poseShooter = getEstimatedPoseInfo(camShooter);
        poseIntake = getEstimatedPoseInfo(camIntake);

        poseShooterActual = poseShooter.getPose2d();
        poseIntakeActual = poseIntake.getPose2d();

        if (poseShooter.getNumOfTags() != 0) {
            m_RobotContainer.m_robotDrive.m_odometry.addVisionMeasurement(poseShooterActual,
                    poseShooter.getTimestampSeconds());
            m_field.getObject("poseShooter").setPose(poseShooter.getPose2d());
        }
        // if (poseIntake.getNumOfTags() != 0) {
        // m_RobotContainer.m_robotDrive.m_odometry.addVisionMeasurement(poseIntakeActual,
        // poseIntake.getTimestampSeconds());
        // m_field.getObject("poseIntake").setPose(poseIntake.getPose2d());
        // }
        m_field.setRobotPose(m_RobotContainer.m_robotDrive.getPose());

    }

    @AutoLogOutput

    public double getIntakeVisionOffset() {
        if (m_RobotContainer.m_elevator.getIntakePos().getHeight() > 14) {
            return 0.66 + Units.inchesToMeters(m_RobotContainer.m_elevator.getIntakePos().getHeight() - 14);
        }
        return 0.66;
    }

    public void updateSmartDashBoard() {

        if (poseShooter.getNumOfTags() != 0) {
            SmartDashboard.putNumber("camShooter x", poseShooter.getPose2d().getX());
            SmartDashboard.putNumber("camShooter y", poseShooter.getPose2d().getY());
        }
        if (poseIntake.getNumOfTags() != 0) {
            SmartDashboard.putNumber("intakeShooter x", poseIntake.getPose2d().getX());
            SmartDashboard.putNumber("intakeShooter y", poseIntake.getPose2d().getY());
        }
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {

        addVisionMeasurement();

    }

    public static class EstimatedPoseInfo {
        private Pose2d pos;
        private double timestampSeconds;
        private double numOfTags;

        public EstimatedPoseInfo(Pose2d pos, double timestampSeconds, double numOfTags) {
            this.pos = pos;
            this.timestampSeconds = timestampSeconds;
            this.numOfTags = numOfTags;
        }

        public Pose2d getPose2d() {
            return pos;
        }

        public double getTimestampSeconds() {
            return timestampSeconds;
        }

        public double getNumOfTags() {
            return numOfTags;
        }

    }

    @AutoLogOutput

    public void testPoseCalculations() {
        double pitch_deg = -15;
        double yaw_deg = 0;
        int tagID = 7;
        TargetCorner corner = new TargetCorner(0, 0);
        PhotonTrackedTarget target = new PhotonTrackedTarget(yaw_deg, pitch_deg, 0, 0, tagID, null, null, 0, null,
                List.of(corner, corner, corner, corner));

        Pose2d robotPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        Pose2d result = getRobotToField(target, camIntake, robotPose);
    }
}