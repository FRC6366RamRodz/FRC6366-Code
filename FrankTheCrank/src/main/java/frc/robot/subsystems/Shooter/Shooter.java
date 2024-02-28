// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;

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

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void ShooterPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Angle", angle);

    angle =
        new Pose3d(0.42, 0.08, 0.52, new Rotation3d(0, getAnlge().getRadians() + Units.degreesToRadians(50), 0));
  }

  public void run3PointArm(boolean intake,boolean speaker,boolean amp,boolean launch,boolean center,boolean wing,boolean autoline) {

    if (intake && getAnlge().getDegrees() > -52 && getAnlge().getDegrees() < -48) {
      ShootSpeed = 0.0;
      IntakeSpeed = 0.7;
      //FeedSpeed = 1;
      shooterAngle = -50;
      launchMode = false;
    } else if (speaker && center) {
      ShootSpeed = 5300;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -9;
      launchMode = true;
    } else if (speaker && wing) {
      ShootSpeed = 6000;
      IntakeSpeed = 0.0;
      //FeedSpeed = 1;
      shooterAngle = -0.3;
      launchMode = true;
    } else if (speaker && !center && !wing) {
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
    } else if (autoline) {
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
  }

  public void advancedShoot(boolean X) {
    double x1, y1, x2, y2, height;
    x1 = 16.45;
    y1 = 5.5;
    x2 = getPose().getX();
    y2 = getPose().getY();
    height = 3.0;
    double Distance = Math.sqrt(Math.pow(x2-x1, 2) + Math.pow(y2-y1, 2));

    double Target;
    Target = Math.atan(2*(height/Distance)-Math.tan(10));
    double velocity, finalAngle;
    if (Target <= Units.degreesToRadians(51)) {
      velocity = 1/Math.cos(Units.radiansToDegrees(Target)) * Math.sqrt((-9.8*Distance)/-Math.abs(Math.tan(Target)));
      finalAngle = Units.radiansToDegrees(-Target);
    } else {
      velocity = (2 * Math.PI * 0.0508 * 3400)/ 60;
      finalAngle = -35;
    }

    double Rpm = (30 * velocity) / Math.PI * 0.0508;
    double finalRPM;
    if (Rpm > 6000) {
      finalRPM = 3400;
    } else {
      finalRPM = Rpm;
    }
    if(X) {
    io.setMotors(finalRPM, finalRPM, 0, finalAngle, 0, 0, false);
    } else {
      io.setMotors(0, 0, 0, -50, 0, 0, false);
    }
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
