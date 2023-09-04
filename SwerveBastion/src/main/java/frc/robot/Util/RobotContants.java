// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/** Add your docs here. */
public class RobotContants {
    public static final double RobotMass = 125 * 0.453592;
    public static final Matter Chassis = new Matter(new Translation3d(0,0,Units.inchesToMeters(48) ), RobotMass);

    public static final double UpperArmLengthinMeter = Units.inchesToMeters(38);
    public static final double LowerArmLengthinMeter = Units.inchesToMeters(15);
    public static final double ArmHightinMeter = Units.inchesToMeters(46);
    public static final double IntakelengthinMeter = Units.inchesToMeters(10);
}
