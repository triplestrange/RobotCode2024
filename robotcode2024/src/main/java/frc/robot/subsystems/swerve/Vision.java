package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class Vision extends SubsystemBase {

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

    public Vision() {
        camLeft = new PhotonCamera("camLeft");
        camRight = new PhotonCamera("camRight");
        camFront = new PhotonCamera("camFront");
        camBack = new PhotonCamera("camBack");

    }

    public void updateSmartDashBoard() {

    }

    public void periodic() {

    }
}
