// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Container.RobotContainer;

/** Add your docs here. */
public class Drive{
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3.0);
    
    private final DriveIo io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(new Rotation2d(), 0.0, 0.0);

    public Drive(DriveIo io) {
        this.io = io;
    }

    
    public void periodicDrive() {
        io.updateInputs(inputs);
        org.littletonrobotics.junction.Logger.getInstance().processInputs("Drive", inputs);

        odometry.update(new Rotation2d(), getLeftPositionMeters(), getRightPositionMeters());
        org.littletonrobotics.junction.Logger.getInstance().recordOutput("Odometry", getPose());
    }

    public void drivePercent(double leftPercent, double rightPercent) {
        io.setVoltage(leftPercent*12.0, rightPercent*12.0);
    }

    public void driveArcade(double xSpeed, double zRotation) {
        var speeds = DifferentialDrive.arcadeDriveIK(xSpeed, zRotation, false);
        io.setVoltage(speeds.left*12.0, speeds.right*12.0);
    }

    public void stop() {
        io.setVoltage(0.0, 0.0);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getLeftPositionMeters() {
        return inputs.leftPositionRad * WHEEL_RADIUS_METERS;
    }

    public double getRightPositionMeters() {
        return inputs.rightPostitionRad * WHEEL_RADIUS_METERS;
    }

    public double getLeftVelocityMeters() {
        return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
    }

    public double getRightVelocityMeters() {
        return inputs.leftVelocityRadPerSec * WHEEL_RADIUS_METERS;
    }


}
