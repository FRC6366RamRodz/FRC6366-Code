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

    angle =
        new Pose3d(0.42, 0.08, 0.52, new Rotation3d(0, getAnlge().getRadians() + Units.degreesToRadians(50), 0));
  }

  public void run3PointArm(boolean intake,boolean speaker,boolean amp,boolean launch,boolean center,boolean wing,boolean autoline) {

    if (intake && getAnlge().getDegrees() > -52 && getAnlge().getDegrees() < -48) {
      ShootSpeed = 0.0;
      IntakeSpeed = 1;
      FeedSpeed = 1;
      shooterAngle = -50;
      launchMode = false;
    } else if (speaker && center) {
      ShootSpeed = 3800;
      IntakeSpeed = 0.0;
      FeedSpeed = 1;
      shooterAngle = -10;
      launchMode = true;
    } else if (speaker && wing) {
      ShootSpeed = 4500;
      IntakeSpeed = 0.0;
      FeedSpeed = 1;
      shooterAngle = -2;
      launchMode = true;
    } else if (speaker && !center && !wing) {
      ShootSpeed = 3400;
      IntakeSpeed = 0.0;
      FeedSpeed = 1;
      shooterAngle = -35;
      launchMode = true;
    } else if (amp) {
      ShootSpeed = 500;
      IntakeSpeed = 0.0;
      FeedSpeed = 1;
      shooterAngle = 50;
      launchMode = true;
    } else if (autoline) {
      ShootSpeed = 3800;
      IntakeSpeed = 0.0;
      FeedSpeed = 1;
      shooterAngle = -40;
      launchMode = true;
    } else {
      ShootSpeed = 0.0;
      IntakeSpeed = 0.0;
      FeedSpeed = 0.0;
      shooterAngle = -50.0;
      launchMode = false;
    }

    boolean limitOff;
    if (LaunchPermision() == 1 && launch && (amp || speaker)) {
      sideSpeed = 0.5;
      limitOff = true;
    } else {
      sideSpeed = 0;
      limitOff = false;
    }

    io.setMotors(ShootSpeed, ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed, limitOff);
  }

  public double LaunchPermision() {
    if (shooterAngle < getAnlge().plus(new Rotation2d(2)).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(2)).getDegrees() && ShootSpeed < getAvrgShootSpd() + 15 && ShootSpeed > getAvrgShootSpd() - 15&& launchMode) {
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
}
