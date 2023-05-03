// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;
import frc.robot.Subsystems.SubDriveTrain;
import frc.robot.util.Constants.Map;


/** Add your docs here. */
public class RobotContainer {

    public static final SubDriveTrain driveTrain = new
            SubDriveTrain(Map.LEFT_FRONT_DRIVETRAIN, Map.LEFT_BACK_DRIVETRAIN,
        Map.RIGHT_FRONT_DRIVETRAIN, Map.RIGHT_BACK_DRIVETRAIN);


}
 