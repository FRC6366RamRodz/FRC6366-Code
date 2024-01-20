// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
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
  public double shooterAngle;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  public void ShooterPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Angle", angle);
  }

  public void teleop(boolean intake, boolean shoot, boolean amp, boolean fire) {
    if (intake) {
      IntakeSpeed = 1;
      FeedSpeed = 0.5;
      shooterAngle = 0;
    } else if (amp) {
      IntakeSpeed = 0;
      FeedSpeed = 0;
      shooterAngle = 80;
    } else if (fire && Math.abs(shooterAngle) > Math.abs(getAnlge())-50 && Math.abs(ShootSpeed) > Math.abs(getAvrgShootSpd())) {
      IntakeSpeed = 0;
      FeedSpeed = 20;
    } else {
      IntakeSpeed = 0;
      FeedSpeed = 0;
      shooterAngle = 0;
    }

    if (shoot) {
      ShootSpeed = 500;
    } else if (amp) {
      ShootSpeed = 200;
    } else {
      ShootSpeed = 0;
    }

    
    io.setMotors(ShootSpeed, -ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed);
    
    angle = new Pose3d(0.42, 0.08, 0.52, new Rotation3d(0, getAnlge(), 0));
    
  }

  public double getAnlge() {
    return Units.degreesToRadians(inputs.anglePosition);
  }

  public double getAvrgShootSpd() {
    return (Math.abs(inputs.BottomVelocity) + Math.abs(inputs.TopVelocity)) / 2;
  }
}
