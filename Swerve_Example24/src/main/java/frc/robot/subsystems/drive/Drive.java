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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.PoseEstimator.TimestampedVisionUpdate;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

//modified from 6328's 2023 example so that it supports talon FX motorControllers
public class Drive extends SubsystemBase {
  private static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.7); //Free Speed, not actual
  private static final double TRACK_WIDTH_X = Units.inchesToMeters(21.125); //width from wheel to wheel, not frame
  private static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.375);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0); // done automatically
  private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS; //done automatically
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged(); //location of usable inputs from gyro
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations()); //math class for odometry
  private Rotation2d lastGyroRotation = new Rotation2d(); //?

  private frc.robot.util.PoseEstimator combinedOdometry = new frc.robot.util.PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002)); //Real Odometry used for all math
  private frc.robot.util.PoseEstimator visionOdometry = new frc.robot.util.PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002)); //Last detected Vision Pose
  private frc.robot.util.PoseEstimator wheelOdometry = new frc.robot.util.PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002)); //Mechanical odometry (no Vision)

  public  SwerveModulePosition[] wheelDeltas; //Location of all 4 modules.

  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0); //create an index for all 4 modules
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic( this::getPose, this::setPose, () -> kinematics.toChassisSpeeds(getModuleStates()), this::runVelocity, 
    new HolonomicPathFollowerConfig(MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()), 
    () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red, this);

    Pathfinding.setPathfinder(new LocalADStarAK());// Pathfind when not starting on an auto path, LocalADStarAK is the Akit compatible one.

    PathPlannerLogging.setLogActivePathCallback((activePath) -> {Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));});//loging stuff
    
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);});//logging stuff
     
  }


  public void periodic() {//hapens the entire time robot is live
    gyroIO.updateInputs(gyroInputs); //updates the logged values
    Logger.processInputs("Drive/Gyro", gyroInputs); //records logged values
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();//stops motor output on disable
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {//log module states even when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    wheelDeltas = new SwerveModulePosition[4];
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
    combinedOdometry.addDriveData(Timer.getFPGATimestamp(), twist);
    // Apply the twist (change since last loop cycle) to the current pose
    wheelOdometry.addDriveData(Timer.getFPGATimestamp(), twist);
    
  }

  

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02); //speeds that demands come from
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds); //translated demand to readable
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED); //normalizes the readable outputs

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
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

  public void checkFrontVision() {

    if (RobotContainer.cameras.getTargetData() != null) {
      Optional<Pose2d> visionPose = RobotContainer.cameras.getEstimatedPose();
      PhotonPipelineResult result = RobotContainer.cameras.getFilteredResult();

      if (visionPose.isPresent()) {       
        visionOdometry.resetPose(visionPose.get());//reports latest vision pose.
        Logger.recordOutput("PhotonResults", result);
        combinedOdometry.resetPose(getPose().interpolate(new Pose2d(visionPose.get().getX(),visionPose.get().getY(), getRotation()), 0.18));//interpolates a position between vision pose and real pose @ stnd dev of 18%
      }
    }
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {//Neo stuff, no clue
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {//returns output from characterization
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
    return combinedOdometry.getLatestPose();
  }

    @AutoLogOutput(key = "Odometry/RobotMech")
  public Pose2d getMechanicalPose() {
    return wheelOdometry.getLatestPose();
  }

    @AutoLogOutput(key = "Odometry/RobotVision")
  public Pose2d getVisionPose() {
    return visionOdometry.getLatestPose();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return combinedOdometry.getLatestPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    combinedOdometry.resetPose(pose);
    wheelOdometry.resetPose(pose);
  }

  /** Adds vision data to the pose esimation. */
  public void addVisionData(List<TimestampedVisionUpdate> visionData) {
    combinedOdometry.addVisionData(visionData);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }


    //for wheel clibration
  public double[] getDrivePosition() {
      return Arrays.stream(modules).mapToDouble(Module::getPositionRad).toArray();
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
