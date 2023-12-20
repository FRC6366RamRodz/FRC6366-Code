// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Drive {
  public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry =
      new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

  public Drive(DriveIO io) {
    this.io = io;
  }

  public void DrivePeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);

    odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
  }

  public void drivePercent(
      double leftPercent, double rightPercent) { // set open loop constant voltage
    io.setVoltage(leftPercent, rightPercent);
  }

  public void driveCurveDrive(double xSpeed, double zRotation, double sens) {
    double forward;
    if (Math.abs(xSpeed) > 0.8) {
      forward = Math.abs(xSpeed);
    } else {
      forward = -Math.abs(xSpeed) + 1;
    }

    double left = MathUtil.clamp((xSpeed - zRotation * sens * forward), -1, 1);
    double right = MathUtil.clamp((xSpeed + zRotation * sens * forward), -1, 1);
    io.setVoltage(left, right);
  }

  public void driveArcade(double xSpeed, double zRotation) {
    var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);
    io.setVoltage(speeds.left, speeds.right);
  }

  public void stop() {
    io.setVoltage(0.0, 0.0);
  }

  /** Returns the current odometry pose in meters. */
  @AutoLogOutput
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionRad * WHEEL_RADIUS;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionRad * WHEEL_RADIUS;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMeters() {
    return inputs.leftVelocity * WHEEL_RADIUS;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMeters() {
    return inputs.rightVelocity * WHEEL_RADIUS;
  }
}
