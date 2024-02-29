// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.NoteVisualizer;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Shooter {

  private Pose3d angle = new Pose3d();

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public double ShootSpeed;
  public double IntakeSpeed;
  public double FeedSpeed;
  public double sideSpeed;
  public double shooterAngle;
  public boolean launchMode;
  public double x1, y1, offset, oldX, oldY;;
  public static InterpolatingDoubleTreeMap shootMap = new InterpolatingDoubleTreeMap();

  private Command noteVisualizer = frc.robot.util.NoteVisualizer.shoot();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void ShooterPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Angle", angle);

    angle = new Pose3d(0.42, 0.08, 0.52, new Rotation3d(0, getAnlge().getRadians() + Units.degreesToRadians(50), 0));

    NoteVisualizer.setRobotPoseSupplier(() -> getPose());

  }
  public void run3PointArm(boolean intake,boolean speaker,boolean amp,boolean launch,boolean center,boolean wing,boolean autoline) {

    if (intake && getAnlge().getDegrees() > -52 && getAnlge().getDegrees() < -48) {
      ShootSpeed = 0.0;
      IntakeSpeed = 0.7;
      //FeedSpeed = 1;
      shooterAngle = -50;
      launchMode = false;
    } else if (speaker && center) {//3.054 Meters from target
      ShootSpeed = 5300;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -9;
      launchMode = true;
    } else if (speaker && wing) {//5.6495 Meters from target
      ShootSpeed = 6000;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -0.3;
      launchMode = true;
    } else if (speaker && !center && !wing) {//1.21 Meters from target
      ShootSpeed = 3400;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -35;
      launchMode = true;
    } else if (amp) {
      ShootSpeed = 3400;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = 50;
      launchMode = true;
    } else if (autoline) {//1.84 Meters from target
      ShootSpeed = 3800;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -20;
      launchMode = true;
    } else {
      ShootSpeed = 0.0;
      IntakeSpeed = 0.0;
      //FeedSpeed = 0.0;
      shooterAngle = -50.0;
      launchMode = false;
    }

    boolean limitOff;
    if (LaunchPermision() == 1 && launch && (amp || speaker || autoline)) {
      sideSpeed = 0.5;
      limitOff = true;
      FeedSpeed = 0.5;
    } else if (intake){
      sideSpeed = 0.0;
      limitOff = false;
      FeedSpeed = 0.25;
    } else {
      sideSpeed = 0;
      limitOff = false;
      FeedSpeed = 0;
    }

    io.setMotors(ShootSpeed, ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed, limitOff);

    if (sideSpeed > 0.1) {
      noteVisualizer.schedule();
    }
  }

  public void advancedShoot(boolean X) {
    shootMap.put(1.25, -35.0);//distance, followed by shot angle
    shootMap.put(1.84, -20.0);//distance, followed by shot angle
    shootMap.put(3.054, -9.0);//distance, followed by shot angle
    shootMap.put(5.6495, -0.3);//distance, followed by shot angle

    Optional<Alliance> ally = DriverStation.getAlliance();

    if (ally.get() == Alliance.Blue){
      x1 = 0; 
      y1 = 5.5;
      offset = Math.PI;
    } else {
      x1 = 16.45;
      y1 = 5.5;
      offset = 0;
    }
    double distance;
    if (getPose() != null) {
      distance = Math.sqrt((Math.pow(getPose().getX() - x1, 2)) + Math.pow(getPose().getY() - y1, 2)); //a^2 + b^2 = c^2 //x2 - x1 = a
    } else {
      distance = 0.0;
    }
    
    
    
    double shootAngle;
    if (X) {
      shootAngle = shootMap.get(distance);
    } else {
      shootAngle = -50;
    }

    Timer time = new Timer();
    time.start();

    if (time.get() > 0.5) {
      oldX = getPose().getX();
      oldY = getPose().getY();
    } 
    
    double xSpeed = getPose().getX()- oldX;
    double ySpeed = getPose().getY() - oldY;

    io.setMotors(0, 0, 0, shootAngle, 0, 0, false);
    
  }

  public double LaunchPermision() {
    if (shooterAngle < getAnlge().plus(new Rotation2d(Units.degreesToRadians(1))).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(Units.degreesToRadians(1))).getDegrees() && ShootSpeed < getAvrgShootSpd() + 40 && ShootSpeed > getAvrgShootSpd() - 40 && getArmSpd() > -0.2 && getArmSpd() < 0.2 && launchMode) {
      return 1;
    } else {
      return 0;
    }
  }

  public double IntakeRumble() {
    if (inputs.intakeLimit && FeedSpeed > 0) {
      return 1;
    } else {
      return 0;
    }
  }

  public Rotation2d getAnlge() {
    return new Rotation2d(Units.degreesToRadians(inputs.anglePosition));
  }

  public double getAvrgShootSpd() {
    return (Math.abs(inputs.TopVelocity) + Math.abs(inputs.BottomVelocity)) / 2;
  }

  public static Pose2d getPose() {
   return RobotContainer.drive.getPose();
  }

  public double getArmSpd() {
    return inputs.angleVelocity;
  }
}
