// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve.inMatchAuto;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
            
                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(1.9, 0.5, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(1.9, 2.1, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(1.9, 1.00, new Rotation2d(180));
                }
                
            } else if (IO.GetAbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(1.9, 2.1, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(1.9, 3.2, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(1.9, 2.7, new Rotation2d(180));
                }

            } else if (IO.GetXbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(1.9, 3.8, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(1.9, 4.9, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(1.9, 4.4, new Rotation2d(180));
                }

            } else if (IO.GetYbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.95, 6.21, new Rotation2d(0));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.19, 7.11, new Rotation2d(90));
                } else {
                    return autoPose = new Pose2d(14.95, 7.47, new Rotation2d(0));
                }
                
            } else {
                return autoPose = new Pose2d(3.84, 4.69, new Rotation2d(180));
            }


        } else {

            if (IO.GetBbuttonOP()) {
            
                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.68, 0.5, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.68, 1.63, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(14.68, 1.06, new Rotation2d(180));
                }
                
            } else if (IO.GetAbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.68, 2.18, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.68, 3.30, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(14.68, 2.75, new Rotation2d(180));
                }

            } else if (IO.GetXbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(14.68, 3.87, new Rotation2d(180));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(14.68, 4.98, new Rotation2d(180));
                } else {
                    return autoPose = new Pose2d(14.68, 4.42, new Rotation2d(180));
                }

            } else if (IO.GetYbuttonOP()) {

                if(IO.GetDpadLeftOP()) {
                    return autoPose = new Pose2d(1.6, 6.21, new Rotation2d(0));
                } else if (IO.GetDpadRightOP()) {
                    return autoPose = new Pose2d(2.49, 7.11, new Rotation2d(90));
                } else {
                    return autoPose = new Pose2d(1.6, 7.47, new Rotation2d(0));
                }

            } else {
                return autoPose = new Pose2d(12.67, 4.69, new Rotation2d(180));
            }


        }

    } 
}
