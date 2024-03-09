// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.NoteVisualizer;

import java.util.Optional;

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
  public boolean autoAim;
  public double x1, y1, offset, oldX, oldY;;
  public static InterpolatingDoubleTreeMap shootMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

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

  @Deprecated
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
    double Climb = 0;

    io.setMotors(ShootSpeed, ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed, limitOff, Climb);

    if (sideSpeed > 0.1) {
      noteVisualizer.schedule();
    }
  }

  public void advancedShoot(boolean SWM, boolean Subwoof, boolean AutoLine, boolean Stage, boolean Wing, boolean Amp, boolean intake, boolean fire, double climb, boolean shootClimb) {
    shootMap.put(1.25, -35.0);//distance, followed by shot angle
    shootMap.put(1.84, -20.0);//distance, followed by shot angle
    shootMap.put(3.054, -9.0);//distance, followed by shot angle
    shootMap.put(5.6495, -0.0);//distance, followed by shot angle
    shootMap.put(3.6068, -7.0);
    shootMap.put(4.0513, -2.8);

    speedMap.put(1.25, 3400.0);//distance, followed by shot speed
    speedMap.put(1.84, 3800.0);//distance, followed by shot speed
    speedMap.put(3.054, 5300.0);//distance, followed by shot speed
    speedMap.put(5.6495, 6000.0);//distance, followed by shot speed
    speedMap.put(3.6068, 5400.0);
    speedMap.put(4.0513, 6000.0);

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
    
    
/*  SWM stuff
    Timer time = new Timer();
    time.start();

    if (time.get() > 0.02) {
      oldX = getPose().getX();
      oldY = getPose().getY();
      time.stop();
      time.reset();
    } 
    
    double xSpeed = -getPose().getX() + oldX;
    double ySpeed = -getPose().getY() + oldY;
*/
    double adjDistance = Math.abs(distance); //+ xSpeed;

    if (SWM && !Subwoof && !AutoLine && !Stage && !Wing && !Amp && !intake) {
      shooterAngle = shootMap.get(adjDistance);
      launchMode = true;
      ShootSpeed = speedMap.get(adjDistance);
      autoAim = true;
    } else if (Subwoof) {
      shooterAngle = -35;
      launchMode = true;
      ShootSpeed = 3800;
      autoAim = false;
    } else if (AutoLine){
      shooterAngle = -20;
      launchMode = true;
      ShootSpeed = 3800;
      autoAim = false;
    } else if (Stage) {
      shooterAngle = -9;
      launchMode = true;
      ShootSpeed = 5300;
      autoAim = false;
    } else if (Wing) {
      shooterAngle = -0.3;
      launchMode = true;
      ShootSpeed = 6000;
      autoAim = false;
    } else if(Amp) {
      shooterAngle = 50;
      launchMode = true;
      ShootSpeed = 1000;
      autoAim = false;
    } else if (intake && !Subwoof && !AutoLine && !Stage && !Wing && !Amp && !SWM) {
      shooterAngle = -50;
      launchMode = false;
      ShootSpeed = 0.0;
      autoAim = false;
    } else if (shootClimb) {
      shooterAngle = 30;
      launchMode = false;
      ShootSpeed = 0.0;
      autoAim = false;
    }else {
      shooterAngle = -50;
      launchMode = false;
      ShootSpeed = 0.0;
      autoAim = false;
    }

    boolean limitOff;
    if(LaunchPermision() == 1 && fire && launchMode) {
      sideSpeed = 0.5;
      FeedSpeed = 0.8;
      limitOff = true;
      IntakeSpeed = 0.0;
    } else if (Amp && !fire) {
      sideSpeed = -0.1;
      limitOff = false;
      FeedSpeed = 0.4;
      IntakeSpeed = 0.0;
    } else if (intake && getAnlge().getDegrees() > -51 && getAnlge().getDegrees() < -49) {
      sideSpeed = -0.1;
      limitOff = false;
      FeedSpeed = 0.4;
      IntakeSpeed = 0.8;
    } else {
      sideSpeed = 0.0;
      FeedSpeed = 0.0;
      limitOff = false;
      IntakeSpeed = 0.0;
    }

    double climber;
    if (RobotContainer.io.getDrAbutton()) {
      climber = climb;
    } else {
      climber = 0;
    }

    io.setMotors(ShootSpeed, ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed, limitOff, climber);
  }

  public double LaunchPermision() {
    if (shooterAngle < getAnlge().plus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && ShootSpeed < getAvrgShootSpd() + 40 && ShootSpeed > getAvrgShootSpd() - 40 && launchMode && autoAim && DriverStation.isTeleop()) {
      return 1;
    }else if (shooterAngle < getAnlge().plus(new Rotation2d(Units.degreesToRadians(1))).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(Units.degreesToRadians(1))).getDegrees() && ShootSpeed < getAvrgShootSpd() + 40 && ShootSpeed > getAvrgShootSpd() - 40 && getArmSpd() > -0.3 && getArmSpd() < 0.3 && launchMode) {
      return 1;
    } else {
      return 0;
    }
  }

  public double IntakeRumble() {
    if (inputs.intakeLimit && IntakeSpeed > 0) {
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
