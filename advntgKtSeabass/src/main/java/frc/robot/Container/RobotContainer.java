// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Container;

import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveHWIntr;
import frc.robot.Subsystems.Drive.DriveIOSim;
import frc.robot.Subsystems.Drive.DriveIo;
import frc.robot.Util.Constants;



/** Add your docs here. */
public class RobotContainer {
    public static Drive Drive;

    public RobotContainer(){
        switch (Constants.currentMode) {

            case REAL:
                Drive = new Drive(new DriveHWIntr());
            break;

            case SIM:
                Drive = new Drive(new DriveIOSim());
            break;

            default:
                Drive = new Drive(new DriveIo() {});
            break;

        }
    }
}
