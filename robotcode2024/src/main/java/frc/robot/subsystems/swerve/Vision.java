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

import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.automations.Shoot;;
public class Vision extends SubsystemBase {

    private SwerveDrive m_SwerveDrive;
    private Shoot m_Shoot;

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



    public Vision(SwerveDrive m_SwerveDrive, Shoot m_shoot) {
        camLeft = new PhotonCamera("camLeft");
        camRight = new PhotonCamera("camRight");
        camShooter = new PhotonCamera("camShooter");
        camIntake = new PhotonCamera("camIntake");

    visionLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camLeft, new Transform3d(new Translation3d(0, -0.32385, 0.5969), new Rotation3d(0,-Math.PI/4,Math.PI/2)));
    visionRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camRight, new Transform3d(new Translation3d(0, 0.32385, 0.5969), new Rotation3d(0,-Math.PI/4, -Math.PI/2)));
    visionShooter = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camShooter, new Transform3d(new Translation3d(0, 0, 0.66), new Rotation3d(Units.degreesToRadians(-2.7), 0, Math.PI)));

        this.m_SwerveDrive = m_SwerveDrive;
        this.m_Shoot = m_shoot;

    }

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator photonPoseEstimator,
            Pose2d prevEstimatedPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedPose);
        return photonPoseEstimator.update();
    }

    public void addVisionMeasurement() {
        poseLeft = getEstimatedPose(visionLeft, m_SwerveDrive.getPose());
        poseRight = getEstimatedPose(visionRight, m_SwerveDrive.getPose());
        poseShooter = getEstimatedPose(visionShooter, m_SwerveDrive.getPose());

        // if (poseLeft.isPresent()) {
        //     if (poseLeft.get().estimatedPose.getX() > 0 && 
        //     poseLeft.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength() && 
        //     poseLeft.get().estimatedPose.getY() > 0 && 
        //     poseLeft.get().estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth())  {
        //         if (
        //             (Math.sqrt(
        //             Math.pow(lastLeftTranslation.getX() - poseLeft.get().estimatedPose.getX(), 2) + 
        //             Math.pow(lastLeftTranslation.getY() - poseLeft.get().estimatedPose.getY(), 2))

        //             / (lastLeftTime - poseLeft.get().timestampSeconds)) 
        //             <= 20)   
        //         {
        //         m_SwerveDrive.m_odometry.addVisionMeasurement(poseLeft.get().estimatedPose.toPose2d(), poseLeft.get().timestampSeconds);
        //         m_field.getObject("poseLeft").setPose(poseLeft.get().estimatedPose.toPose2d());
        //         }
        //     }

        // }

        // if (poseRight.isPresent()) {
        //     if (poseRight.get().estimatedPose.getX() > 0 && 
        //     poseRight.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength() && 
        //     poseRight.get().estimatedPose.getY() > 0 && 
        //     poseRight.get().estimatedPose.getY() < aprilTagFieldLayout.getFieldWidth())  {
        //         if (
        //             (Math.sqrt(
        //             Math.pow(lastRightTranslation.getX() - poseRight.get().estimatedPose.getX(), 2) + 
        //             Math.pow(lastRightTranslation.getY() - poseRight.get().estimatedPose.getY(), 2))

        //             / (lastRightTime - poseRight.get().timestampSeconds)) 
        //             <= 20)   
        //         {

        //             m_SwerveDrive.m_odometry.addVisionMeasurement(poseRight.get().estimatedPose.toPose2d(), poseRight.get().timestampSeconds);
        //             m_field.getObject("poseRight").setPose(poseRight.get().estimatedPose.toPose2d());
        //             }
        //         }

        //     } 

        if (poseShooter.isPresent() && poseShooter.get().estimatedPose.getX() > 0 && poseShooter.get().estimatedPose.getX() < aprilTagFieldLayout.getFieldLength())  {
            if (new Translation2d(poseShooter.get().estimatedPose.getTranslation().getX(), poseShooter.get().estimatedPose.getY() ).minus(new Translation2d(m_Shoot.speakerTranslation3d.getX(), m_Shoot.speakerTranslation3d.getY())).getNorm() <= 5)
            m_SwerveDrive.m_odometry.addVisionMeasurement(poseShooter.get().estimatedPose.toPose2d(), poseShooter.get().timestampSeconds);
            m_field.getObject("poseShooter").setPose(poseShooter.get().estimatedPose.toPose2d());

        }

        if (poseLeft.isPresent())  {
            lastLeftTranslation = new Translation2d(poseLeft.get().estimatedPose.getY(), poseLeft.get().estimatedPose.getY());
            lastLeftTime = poseLeft.get().timestampSeconds;
        }

        if (poseRight.isPresent())  {
            lastRightTranslation = new Translation2d(poseRight.get().estimatedPose.getY(), poseRight.get().estimatedPose.getY());
            lastRightTime = poseRight.get().timestampSeconds;
        }

        m_field.setRobotPose(m_SwerveDrive.getPose());



        
    }

    public void updateSmartDashBoard() {
        if (poseLeft != null && poseLeft.isPresent()) {
            SmartDashboard.putNumber("camleft x", poseLeft.get().estimatedPose.getX());
            SmartDashboard.putNumber("camleft y", poseLeft.get().estimatedPose.getY());
        }

          if (poseRight != null && poseRight.isPresent())   {
        SmartDashboard.putNumber("camright x", poseRight.get().estimatedPose.getX());
        SmartDashboard.putNumber("camright y", poseRight.get().estimatedPose.getY());
        }

          if (poseShooter != null && poseShooter.isPresent())   {
        SmartDashboard.putNumber("camShooter x", poseShooter.get().estimatedPose.getX());
        SmartDashboard.putNumber("camShooter y", poseShooter.get().estimatedPose.getY());
        }

        SmartDashboard.putData("Field", m_field);
    }

    public void periodic() {

        addVisionMeasurement();

    }
}