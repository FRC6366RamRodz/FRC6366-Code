// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
  }

  public void teleop(
      boolean intake, boolean shoot, boolean amp, boolean fire, boolean mod1, boolean mod2) {
    if (intake && shooterAngle < 2 && shooterAngle > -2) {
      IntakeSpeed = 1;
      shooterAngle = 0;
    } else if (shoot && mod1) {
      IntakeSpeed = 0;
      shooterAngle = 50;
    } else if (shoot && mod2) {
      IntakeSpeed = 0;
      shooterAngle = 30;
    } else if (amp) {
      IntakeSpeed = 0;
      shooterAngle = 80;
    } else {
      IntakeSpeed = 0;
      shooterAngle = 0;
    }

    if (intake && shooterAngle < 2 && shooterAngle > -2) {
      FeedSpeed = 0.4;
      sideSpeed = 0;
    } else if (fire
        && shooterAngle < Units.radiansToDegrees(getAnlge()) + 2
        && shooterAngle > Units.radiansToDegrees(getAnlge()) - 2
        && ShootSpeed < getAvrgShootSpd() + 10
        && ShootSpeed > getAvrgShootSpd() - 10
        && launchMode) {
      FeedSpeed = 1;
      sideSpeed = 5000;
    } else {
      FeedSpeed = 0;
      sideSpeed = 0;
    }

    if (shoot && mod1) {
      ShootSpeed = 1000;
      launchMode = true;
    } else if (shoot && mod2) {
      ShootSpeed = 800;
      launchMode = true;
    } else if (shoot) {
      ShootSpeed = 500;
      launchMode = true;
    } else if (amp) {
      ShootSpeed = 200;
      launchMode = true;
    } else {
      ShootSpeed = 0;
      launchMode = false;
    }

    io.setMotors(-ShootSpeed, -ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed);

    angle = new Pose3d(0.42, 0.08, 0.52, new Rotation3d(0, getAnlge(), 0));
  }

  public void advancedTeleop(boolean shoot, boolean fire, boolean intake, double distanceX) {
    double ShootSpeed;
    Rotation2d shooterAngle;
    double IntakeSpeed;

    if (shoot) {
      shooterAngle =
          new Rotation2d(
                  (Math.asin(-9.8 - distanceX * 2))
                      / 2
                      * ((2 * Math.PI * 0.0508 * getAvrgShootSpd()) / 60))
              .minus(new Rotation2d(Units.degreesToRadians(45)));
      ShootSpeed = 2500;
    } else {
      shooterAngle = new Rotation2d(0);
      ShootSpeed = 0;
    }

    if (fire
        && shooterAngle.getDegrees() < Units.radiansToDegrees(getAnlge()) + 2
        && shooterAngle.getDegrees() > Units.radiansToDegrees(getAnlge()) - 2
        && ShootSpeed < getAvrgShootSpd() + 10
        && ShootSpeed > getAvrgShootSpd() - 10
        && launchMode) {
      FeedSpeed = 1;
      sideSpeed = 1000;
    } else {
      FeedSpeed = 0;
      sideSpeed = 0;
    }

    if (shoot) {
      launchMode = true;
    } else {
      launchMode = false;
    }

    if (intake && shooterAngle.getDegrees() < 2 && shooterAngle.getDegrees() > -2) {
      IntakeSpeed = 1;
    } else {
      IntakeSpeed = 0;
    }

    io.setMotors(
        ShootSpeed, ShootSpeed, FeedSpeed, shooterAngle.getDegrees(), IntakeSpeed, sideSpeed);
  }

  public double LaunchPermision() {
    if (shooterAngle < Units.radiansToDegrees(getAnlge()) + 2
        && shooterAngle > Units.radiansToDegrees(getAnlge()) - 2
        && ShootSpeed < getAvrgShootSpd() + 15
        && ShootSpeed > getAvrgShootSpd() - 15
        && launchMode) {
      return 1;
    } else {
      return 0;
    }
  }

  public double getAnlge() {
    return Units.degreesToRadians(inputs.anglePosition);
  }

  public double getAvrgShootSpd() {
    return (Math.abs(inputs.TopVelocity) + Math.abs(inputs.BottomVelocity)) / 2;
  }
}
