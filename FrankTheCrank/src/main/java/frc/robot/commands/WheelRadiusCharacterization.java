// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class WheelRadiusCharacterization extends Command {
  private double radius = Drive.DRIVE_BASE_RADIUS;
  private double lastGyro;
  private double accumGyroYawRads = 0.0;
  private double[] wheelStart;
  private final Drive drive;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);
  private double currentEffectiveWheelRadius = 0.0;

  /** Creates a new WheelRadiusCharacterization. */
  public WheelRadiusCharacterization(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequiremen2ts() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastGyro = drive.getRotation().getRadians();
    accumGyroYawRads = 0;
    wheelStart = drive.getDrivePosition();

    omegaLimiter.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.1 * drive.getMaxAngularSpeedRadPerSec(), drive.getRotation()));

  // Get yaw and wheel positions
  accumGyroYawRads += MathUtil.angleModulus(drive.getRotation().getRadians() - lastGyro);
  lastGyro = drive.getRotation().getRadians();
  double averageWheelPosition = 0.0;
  double[] wheelPositiions = drive.getDrivePosition();
  for (int i = 0; i < 4; i++) {
    averageWheelPosition += Math.abs(wheelPositiions[i] - wheelStart[i]);
  }
  averageWheelPosition /= 4.0;

  currentEffectiveWheelRadius = (accumGyroYawRads * radius) / averageWheelPosition;
  Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
  Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
  Logger.recordOutput("Drive/RadiusCharacterization/CurrentWheelRadiusInches", Units.metersToInches(currentEffectiveWheelRadius));
    //wheel radius (meters) = gyro delta (radians) * drive base radius (meters) / wheel position delta (radians)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println("Effective Wheel Radius: " + Units.metersToInches(currentEffectiveWheelRadius) + " inches");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
