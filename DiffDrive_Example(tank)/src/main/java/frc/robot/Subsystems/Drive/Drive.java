// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.ReplanningConfig;

/** Add your docs here. */
public class Drive implements Subsystem {
  public static final double WHEEL_RADIUS = Units.inchesToMeters(3.0);
  public static final double MotorKV = 473;
  public static final double GearRatio = 13;

  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(28));

  public Drive(DriveIO io) {
    this.io = io;

    AutoBuilder.configureRamsete(this::getPose, this::setPose, () -> kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(inputs.leftVelocity, inputs.rightVelocity)), this::runVelocity, new ReplanningConfig(),   () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this);
  }

  public void DrivePeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive", inputs);
    
    odometry.update(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters());
    kinematics.toTwist2d(inputs.leftPositionMeter, inputs.rightPositionMeter);
  }

  public void drivePercent(
      double leftPercent, double rightPercent) { // set open loop constant voltage
    io.setVoltage(leftPercent, rightPercent);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    // 5600 * 0.0762 = M
    //5600 = 473 * 12
    //(473 * 12) * 0.0762 = M
    //12 = M / (473 * 0.0762)
    //manually divide by the max set speed to achieve percentage.
    io.setVoltage(wheelSpeeds.leftMetersPerSecond* GearRatio/(MotorKV * WHEEL_RADIUS*2), wheelSpeeds.rightMetersPerSecond * GearRatio/(MotorKV * WHEEL_RADIUS*2));
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
  @AutoLogOutput (key = "Odometry/Robot")
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    odometry.resetPosition(inputs.gyroYaw, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionMeter;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionMeter;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMeters() {
    return inputs.leftVelocity;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMeters() {
    return inputs.rightVelocity;
  }
}
