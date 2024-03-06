// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(Drive drive,DoubleSupplier xSupplier,DoubleSupplier ySupplier,DoubleSupplier omegaSupplier,BooleanSupplier point, BooleanSupplier speak) {
    return Commands.run( () -> {
          Optional<Alliance> ally = DriverStation.getAlliance();
          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection;
          if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
              linearDirection = new Rotation2d(-xSupplier.getAsDouble(), -ySupplier.getAsDouble());
            } else {
              linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            }
          } else {
            linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          }

          double omega;
          if (NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("tv").getDouble(0) == 1 && point.getAsBoolean() && NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("tl").getDouble(0)!= 0) {
            omega = (NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("tx").getDouble(0) / -26)* 0.3 + (linearMagnitude * 0.2);
          } else if(speak.getAsBoolean()){
            double x1, y1, offset;
            if (ally.get() == Alliance.Blue){
              x1 = 0; 
              y1 = 5.4;
              offset = Math.PI;
            } else {
              x1 = 16.45;
              y1 = 5.6;
              offset = 0;
            }
           

            double x2 = getPose().getX(), y2 = getPose().getY();
            double x = x2 - x1;
            double y = y2 - y1;
            double theta = Math.atan(y/x);
            try (PIDController error = new PIDController(1.4 + linearMagnitude * 0.4, 0.0, 0.001)) {
              omega = error.calculate(getPose().getRotation().minus(new Rotation2d(offset)).getRadians(), theta);
            }

           
          }else {
            omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity;
          linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  drive.getRotation()));
        },
        drive);
  }

  public static Pose2d getPose() {
   return RobotContainer.drive.getPose();
  }
}
