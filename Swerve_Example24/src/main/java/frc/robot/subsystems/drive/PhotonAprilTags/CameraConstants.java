// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.PhotonAprilTags;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** replace camera positions with real camera positions */
public class CameraConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.005;
    public static final double thetaStdDevCoefficient = 0.01;
    public static final Transform3d FrontLeftCam = new Transform3d(Units.inchesToMeters(8.875), Units.inchesToMeters(10.5), Units.inchesToMeters(8.25), new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0).rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0))));
    public static final Transform3d FrontRightCam = new Transform3d(Units.inchesToMeters(8.875), Units.inchesToMeters(10.5), Units.inchesToMeters(8.25), new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0).rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0))));
    public static final Transform3d RearCam = new  Transform3d(Units.inchesToMeters(-16.0), Units.inchesToMeters(-12.0), Units.inchesToMeters(8.5), new Rotation3d(0.0, Units.degreesToRadians(-33.75), 0.0).rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(176.386))));

}
