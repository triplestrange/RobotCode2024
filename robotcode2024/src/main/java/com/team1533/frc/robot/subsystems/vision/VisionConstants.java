package com.team1533.frc.robot.subsystems.vision;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {

    public static final Vector<N3> STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 1);
    public static final Vector<N3> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(2, 2, 1000000000);


    public static final Pose3d  shooterCamOffset = new Pose3d(new Translation3d(0, 0, 0.66),
                    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-14.5), Units.degreesToRadians(182)));
    public static final Pose3d  intakeCamOffset = new Pose3d(new Translation3d(Units.inchesToMeters(5), 0, Units.inchesToMeters(14)),
                    new Rotation3d(Math.PI / 2.0, -Units.degreesToRadians(4.5), 0));


}