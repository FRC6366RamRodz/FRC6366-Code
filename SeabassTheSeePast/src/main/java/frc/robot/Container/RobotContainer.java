// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import frc.robot.Subsystems.SubDriveTrain;
import frc.robot.Util.Constants.DT_Map;

/** Add your docs here. */
public class RobotContainer {
    public static final SubDriveTrain DriveTrain = new 
    SubDriveTrain(DT_Map.leftFront, DT_Map.rightFront, DT_Map.leftRear, DT_Map.rightRear, DT_Map.stinger, DT_Map.strafe, DT_Map.stingerSolenoid, DT_Map.strafeSolenoid);
}
