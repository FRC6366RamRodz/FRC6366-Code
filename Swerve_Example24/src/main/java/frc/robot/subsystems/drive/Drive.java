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

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.7);
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(28);
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(28);
  private static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d lastGyroRotation = new Rotation2d();

  private frc.robot.util.PoseEstimator poseEstimator = new frc.robot.util.PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic( this::getPose, this::setPose, () -> kinematics.toChassisSpeeds(getModuleStates()), this::runVelocity, 
    new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()), 
    () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    
    PathPlannerLogging.setLogActivePathCallback((activePath) -> {Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));});
    
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);});
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      wheelDeltas[i] = modules[i].getPositionDelta();
    }
    // The twist represents the motion of the robot since the last
    // loop cycle in x, y, and theta based only on the modules,
    // without the gyro. The gyro is always disconnected in simulation.
    var twist = kinematics.toTwist2d(wheelDeltas);
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last loop cycle.
      twist = new Twist2d(twist.dx, twist.dy, gyroInputs.yawPosition.minus(lastGyroRotation).getRadians()); lastGyroRotation = gyroInputs.yawPosition;
    }
    poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
    // Apply the twist (change since last loop cycle) to the current pose
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0) != 0) {
      checkVisionMeasurements();
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  //this is a modified file from 4481s 2023 code release
  public void checkVisionMeasurements() {
    double[] limelightPoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[] {0});
    double[] limelightTargetSpacePoseDoubleTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[] {0});
    double tagAreaTop = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    // Check if the limelight rejects the position because it is too far away
    // This has to be done with the targetspace pose instead of the wpilibblue pose, because
    // limelight still
    // updates the wpilibblue pose when it shouldn't update according to the pipeline settings
    double sumTop = 0;
    for (double value : limelightTargetSpacePoseDoubleTop) {
      sumTop += value;
    }
    boolean topInRange = sumTop != 0;
    // Stop if limelight array is not full
    // Stop if translation X or Y is not in field
    // Stop if translation Z is negative
    double[] limelightPoseDouble;
    double tagArea;
    boolean topValid = topInRange && (limelightPoseDoubleTop.length > 5 && limelightPoseDoubleTop[0] > 0.75 && limelightPoseDoubleTop[0] < /*FIELD_LENGTH*/ 16.5 - 0.75 && limelightPoseDoubleTop[1] > 0.3 && limelightPoseDoubleTop[1] < /*FIELD_WIDTH*/ 8.20 - 0.3 && limelightPoseDoubleTop[2] >= 0);

    if (topValid) {
      limelightPoseDouble = limelightPoseDoubleTop;
      tagArea = tagAreaTop;
    } else {
      limelightPoseDouble = limelightPoseDoubleTop;
      tagArea = 0;
    }

    Pose2d limelightPose;
    double[] limeLightID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
    if (limeLightID.length >= 2) {
      limelightPose =
        new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));
    } else {
      limelightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), gyroInputs.yawPosition);
    
    }
    SmartDashboard.putNumberArray("DT/vision/LL-top pose", new Double[] {limelightPoseDoubleTop[0], limelightPoseDoubleTop[1], limelightPoseDoubleTop[5]});
    SmartDashboard.putNumberArray("DT/vision/LL final pose",new Double[] {limelightPoseDouble[0], limelightPoseDouble[1], limelightPoseDouble[5]});

    try {
      double[] limelightTagCorners = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tcornxy").getDoubleArray(new double[] {0});
      double[] limelightTagCornersX = new double[4];
      double[] limelightTagCornersY = new double[4];
      for (int i = 0; i < 4; i++) {
        limelightTagCornersX[i] = limelightTagCorners[i * 2];
        limelightTagCornersY[i] = limelightTagCorners[i * 2 + 1];
      }
      SmartDashboard.putNumberArray("DT/vision/LL-top tag corners x", limelightTagCornersX);
      SmartDashboard.putNumberArray("DT/vision/LL-top tag corners y", limelightTagCornersY);
    } catch (Exception e) {
    }

    // Add estimator trust using april tag area (standard Deviations)
    double stdX = 0.15 * ((1-tagArea) * 1000);
    double stdY = stdX;
    SmartDashboard.putNumber("DT/vision/april tag std X", stdX);
    SmartDashboard.putNumber("DT/vision/april tag std Y", stdY);

    // Add limelight latency
    double limelightLatency = limelightPoseDouble[6] / 1000;

    List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
    visionUpdates.add(new TimestampedVisionUpdate(Timer.getFPGATimestamp() - limelightLatency, limelightPose, VecBuilder.fill(stdX, stdY, stdY * 10)));//stdx stdy stdRotation
    if (topValid) {
      poseEstimator.addVisionData(visionUpdates);
    }
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /** Adds vision data to the pose esimation. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    poseEstimator.addVisionData(visionData);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  public void updateOdoWithVision(boolean h) {
    double[] limelightPoseDouble = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[] {0});
    Pose2d limeLightPose = new Pose2d(new Translation2d(limelightPoseDouble[0], limelightPoseDouble[1]), Rotation2d.fromDegrees(limelightPoseDouble[5]));

    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1 && !h) {
      poseEstimator.resetPose(limeLightPose);
    } else if (h && NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
      poseEstimator.resetPose(new Pose2d(limeLightPose.getX(), limeLightPose.getY(), getRotation()));
    }
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
}
