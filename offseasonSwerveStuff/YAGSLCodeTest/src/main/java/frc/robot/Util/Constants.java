// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/** Add your docs here. */
public class Constants {

    public static final double ROBOT_MASS = 45 * 0.453592; //lbs * kg per lbs
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms sprk max velocity lag
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(14)), LOOP_TIME);

    public static class DT_SET {

    public static final double MX_SPD = 1;
    public static final double DedBnd = .1;
    
    }
}
