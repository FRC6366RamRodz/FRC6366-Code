// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.inMatchAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.IO;

/** Add your docs here. */
public class InMatchAuto {
    private Pose2d autoPose;

    public InMatchAuto() {}

    public Pose2d autoSelect() {
        boolean isBlue;

        if (DriverStation.getAlliance().equals(Alliance.Blue)) {
            isBlue = true;
        } else {
            isBlue = false;
        }



        if (isBlue) {


            if (IO.GetBbuttonOP()) {
            
                if(IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2, 0.5, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2, 1.63, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2, 1.00, new Rotation2d(Units.degreesToRadians(180)));
                }
                
            } else if (IO.GetAbuttonOP()) {

                if(IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2, 2.2, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2, 3.3, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2, 2.7, new Rotation2d(Units.degreesToRadians(180)));
                }

            } else if (IO.GetXbuttonOP()) {

                if(IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2, 3.86, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2, 5, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2, 4.45, new Rotation2d(Units.degreesToRadians(180)));
                }

            } else if (IO.GetYbuttonOP()) {

                if(IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.5, 6.21, new Rotation2d(0));
                } else if (IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.19, 7.11, new Rotation2d(Units.degreesToRadians(90)));
                } else {
                    return autoPose = new Pose2d(14.5, 7.0, new Rotation2d(0));
                }
                
            } else {
                return autoPose = new Pose2d(3.84, 4.69, new Rotation2d(Units.degreesToRadians(180)));
            }


        } else {

            if (IO.GetBbuttonOP()) {
            
                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2.0, 4.1, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2.0, 3.0, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2.0, 3.5, new Rotation2d(Units.degreesToRadians(180)));
                }
                
            } else if (IO.GetAbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2.0, 5.8, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2.0, 4.69, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2.0, 5.2, new Rotation2d(Units.degreesToRadians(180)));
                }

            } else if (IO.GetXbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(2.0, 7.48, new Rotation2d(Units.degreesToRadians(180)));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2.0, 6.36, new Rotation2d(Units.degreesToRadians(180)));
                } else {
                    return autoPose = new Pose2d(2.0, 6.9, new Rotation2d(Units.degreesToRadians(180)));
                }

            } else if (IO.GetYbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.49, 1.8, new Rotation2d(0));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.24, 0.94, new Rotation2d(Units.degreesToRadians(-90)));
                } else {
                    return autoPose = new Pose2d(14.49, 0.56, new Rotation2d(0));
                }

            } else {
                return autoPose = new Pose2d(3.7, 3.2, new Rotation2d(Units.degreesToRadians(180)));
            }


        }

    } 
}
