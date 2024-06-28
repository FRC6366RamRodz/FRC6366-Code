// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.NoteVisualizer;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  public double x1, y1, offset, oldX, oldY, distance, PassDIstance;
  public static InterpolatingDoubleTreeMap shootMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap PassMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap PassSPedMap = new InterpolatingDoubleTreeMap();
  public Timer oneLaunch = new Timer();
  public int shotNumber = 0;
  public Timer shotRecorder = new Timer();
  public ArrayList<Double> ShotDistance = new ArrayList<>();
  private LoggedDashboardChooser<String> ShooterMode = new LoggedDashboardChooser<>("ShooterMode"); 
  private static final String OffSeason = "Offseason";
  private static final String Event = "Event";
  private double ManualAngle;
  private double ManualSpeed;
 

  private Command noteVisualizer = frc.robot.util.NoteVisualizer.shoot();
  
  private Pose2d oldpose = new Pose2d();
  private Timer time = new Timer();

  public Shooter(ShooterIO io) {
    this.io = io;

    ShooterMode.addDefaultOption("Event", Event);
    ShooterMode.addOption("OffSeason", OffSeason);

    SmartDashboard.putNumber("SetAngle", -50);
    SmartDashboard.putNumber("SetSpeed", 0);
  }

  public void ShooterPeriodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Angle", angle);

    angle = new Pose3d(0.36, 0.09, 0.585, new Rotation3d(0, getAnlge().getRadians() + Units.degreesToRadians(50), 0));

    NoteVisualizer.setRobotPoseSupplier(() -> getPose()); //optional visualizer for testing.
    NoteVisualizer.launcherTransform =  new Transform3d(0.2, 0, 0.7, new Rotation3d(0.0, Units.degreesToRadians(getAnlge().getDegrees()-20), 0.0));//adjust origin point of notes here
    NoteVisualizer.shotSpeed = (2 * Math.PI * 0.0508 * Math.abs(inputs.TopVelocity)) / 60 / 1; //final divide is for slow mode //speed the shot should be at (2*PI*WheelRadius*RPM)/60 also known as the tip speed equation.

    Logger.recordOutput("Shooter/ShootDistance", Math.abs(distance));//log distance to speaker
    Logger.recordOutput("Shooter/PassDistance", Math.abs(PassDIstance));//log distance to Pass Target
    Logger.recordOutput("Shooter/shotDistances",  ShotDistance.toString());
  }

  public void advancedShoot(boolean SWM, boolean Subwoof, boolean AutoLine, boolean Stage, boolean Wing, boolean Amp, boolean intake, boolean fire, double climb, boolean pass, boolean autoShoot, double error) {
    //interpolation map, used to create setpoints on the fly, (Shoot From Anywhere) more data points means more acuracy. If a shot from a certian region is consistantly bad, add a point in that region.
    shootMap.put(1.25, -35.5);//distance, followed by shot angle //subwoof 
    shootMap.put(1.84, -21.0);//distance, followed by shot angle //auto line
    shootMap.put(2.7, -11.5);//distance, followed by shot angle //stage
    shootMap.put(5.6495, 2.8);//distance, followed by shot angle //wing
    shootMap.put(3.7338, -4.0);//distance, followed by shot angle //wing
    shootMap.put(4.8, 2.0);//distance, followed by shot angle //wing
    shootMap.put(6.0, 4.0);//distance, followed by shot angle //wing

    //if shots are bouncing out lower the relevant shot speed
    speedMap.put(1.25, 2000.0);//distance, followed by shot speed //subwoof 
    speedMap.put(1.84, 2900.0);//distance, followed by shot speed //auto line
    speedMap.put(3.054, 4500.0);//distance, followed by shot speed //stage
    speedMap.put(4.8, 5000.0);//distance, followed by shot angle //wing
    speedMap.put(5.6495, 5200.0);//distance, followed by shot speed //wing
    speedMap.put(6.0, 5700.0);//distance, followed by shot angle //wing

    PassMap.put(10.2, -30.0);

    PassSPedMap.put(10.2, 2700.0);

    Optional<Alliance> ally = DriverStation.getAlliance();


    //new swm stuff
        
    
    time.start();
    if (time.get() > 0.05) {
      oldpose = getPose();

      time.reset();
    }

    Translation2d poseDelta = new Translation2d(oldpose.getX() - getPose().getX(), oldpose.getY()-getPose().getY());
    //end new swm stuff

    
    if (ally.get() == Alliance.Blue){
      x1 = 0; 
      y1 = 5.5;
      offset = Math.PI;
    } else {
      x1 = 16.45;
      y1 = 5.5;
      offset = 0;
    }
    if (getPose() != null) {
      distance = Math.sqrt((Math.pow((getPose().getX()+poseDelta.getX()) - x1, 2)) + Math.pow((getPose().getY()+poseDelta.getY()) - y1, 2)); //a^2 + b^2 = c^2 //x2 - x1 = a
    } else {
      distance = 0.0;
    }

    double x1p, y1p;
            if (ally.get() == Alliance.Blue){
              x1p = 0.62; 
              y1p = 7.33;
            } else {
              x1p = 15.83;
              y1p = 7.53;
            }
    if (getPose() != null) {
      PassDIstance = Math.sqrt((Math.pow(getPose().getX() - x1p, 2)) + Math.pow(getPose().getY() - y1p, 2)); //a^2 + b^2 = c^2 //x2 - x1 = a
    } else {
      PassDIstance = 0.0;
    }

    
/*  SWM stuff //doesnt work needs redone
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
    boolean speaker;
    
    switch (ShooterMode.get()) {
      case Event:
        //primary if loop for all logic in this subsystem   
        if (SWM && !Subwoof && !AutoLine && !Stage && !Wing && !Amp && !intake) {//logic for SWM, needs !case, (Not True) to prevent ruining existing call order
          shooterAngle = shootMap.get(adjDistance);//shootMap.get(adjDistance) translates to get shoot map, interpolated to distance of adjDistance.
          launchMode = true;//a protections case for shooting statements
          ShootSpeed = speedMap.get(adjDistance);
          autoAim = true; //launch permision needed to be sligthly altered for automated aiming.
          speaker = true;
        } else if (Subwoof) {
          shooterAngle = -35.5;//fallback setpoints
          launchMode = true;
          ShootSpeed = 2800;
          autoAim = false;
          speaker = true;
        } else if (AutoLine){
          shooterAngle = -20;
          launchMode = true;
          ShootSpeed = 2900;
          autoAim = false;
          speaker = true;
        } else if (Stage) {
          shooterAngle = -11;
          launchMode = true;
          ShootSpeed = 4000;
          autoAim = false;
          speaker = true;
        } else if (Wing) {
          shooterAngle = 2.0;
          launchMode = true;
          ShootSpeed = 5000;
          autoAim = false;
          speaker = true;
        } else if(Amp) {
          shooterAngle = 35;
          launchMode = true;
          ShootSpeed = 1000;
          autoAim = false;
          speaker = false;
        } else if ((intake || RobotContainer.io.getDrRTrigger()) && !Subwoof && !AutoLine && !Stage && !Wing && !Amp && !SWM) {
          shooterAngle = -50;
          launchMode = false;
          ShootSpeed = 0.0;
          autoAim = false;
          speaker = false;
        } else if (pass) {
          shooterAngle = -30;
          launchMode = true;
          ShootSpeed = 2700;
          autoAim = false;
          speaker = false;
        } else if (RobotContainer.io.getOPLYDown()) {
          shooterAngle = PassMap.get(Math.abs(PassDIstance));
          launchMode = true;
          ShootSpeed = PassSPedMap.get(Math.abs(PassDIstance));
          autoAim = true;
          speaker = false;
        }else if (autoShoot) {
          shooterAngle = shootMap.get(adjDistance);
          launchMode = true;
          ShootSpeed = speedMap.get(adjDistance);
          autoAim = true;
          speaker = true;
        } else if (!launchMode && DriverStation.isAutonomous()) {
          shooterAngle = -50;
          launchMode = false; //prevents launch permision from being given when not in a shot position
          ShootSpeed = 2000 + adjDistance * 10;//speed the shooter up when within 5m of speaker.
          autoAim = false;
          speaker = false;
        } else if(inputs.intakeLimit && !launchMode) {
          shooterAngle = -50;
          launchMode = false; //prevents launch permision from being given when not in a shot position
          ShootSpeed = 2800;//speed the shooter up
          autoAim = false;
          speaker = false;
        }else {
          shooterAngle = -50;//resting state
          launchMode = false;
          ShootSpeed = 0.0;
          autoAim = false;
          speaker = false;
        }
        break;
    
      default:
      
        ManualAngle = SmartDashboard.getNumber("SetAngle", -50);
        ManualSpeed = SmartDashboard.getNumber("SetSpeed", 0);
        if (ManualAngle < -50) {
          ManualAngle = 50;
        } else if (ManualAngle > 85) {
          ManualAngle = 85;
        }

        if (ManualSpeed > 5999) {
          ManualSpeed = 5999;
        } else if (ManualSpeed < 0) {
          ManualSpeed = 0;
        }


        if (SWM) {
          shooterAngle = ManualAngle;
          launchMode = true;
          ShootSpeed = ManualSpeed;
          autoAim = false;
          speaker = false;
        } else if (Amp) {
          shooterAngle = -50;
          launchMode = true;
          ShootSpeed = -2000;
          autoAim = false;
          speaker = false;
        } else if (intake) {
          shooterAngle = -50;
          launchMode = false;
          ShootSpeed = 0.0;
          autoAim = false;
          speaker = false;
        } else {
          shooterAngle = -50;
          launchMode = false;
          ShootSpeed = 0.0;
          autoAim = false;
          speaker = false;
        }

        break;
    }
    boolean limitOff;
    if(LaunchPermision() == 1 && fire && launchMode) {//Indexer controlls
      sideSpeed = 0.9;
      FeedSpeed = 0.8;
      limitOff = true;
      IntakeSpeed = 0.0;
          oneLaunch.start();
          if (oneLaunch.get() < 0.1 && speaker) {// when pressed for notVisualixer
            noteVisualizer.schedule();
          }  
    } else if (Amp && !fire) { //Note prematurly feeds into shooter wheels without.
      sideSpeed = -0.1; //spins side wheel slowly back to prevent droping into shooter wheels.
      limitOff = false;
      FeedSpeed = 0.4; //honestly dont know (Best not to touch in that case)
      IntakeSpeed = 0.0;
    } else if ((intake || RobotContainer.io.getDrRTrigger()) && getAnlge().getDegrees() > -51 && getAnlge().getDegrees() < -49) {
      sideSpeed = -0.2;//anti overfeed when intakeing
      limitOff = false;
      FeedSpeed = 0.6; //huge controll over intake speed, but need to be slow enough the limit goes off
      IntakeSpeed = 0.95;
    } else if (autoShoot && error < Units.degreesToRadians(9) && LaunchPermision() == 1) { //case for automatic shooting. Extra case for robot pointing was needed
      sideSpeed = 0.9;
      FeedSpeed = 0.5;
      limitOff = true;
      IntakeSpeed = 0.0;
          oneLaunch.start();
          if (oneLaunch.get() < 0.1 && speaker) {
            noteVisualizer.schedule();
          }
    }else {
      sideSpeed = 0.0;
      FeedSpeed = 0.0;
      limitOff = false;
      IntakeSpeed = 0.0;
      oneLaunch.stop();
      oneLaunch.reset();
    }

    double climber;
    if (RobotContainer.io.getDrAbutton()) {//release permision, prevents premature movement
      climber = climb;
    } else {
      climber = 0;
    }

    if(RobotContainer.io.getDpad() == 0) {//unjam stuff. sequencing is weird. Dpad Up, Down, Up, Hold Down until not is ejected.
      FeedSpeed = -1;
    } else if (RobotContainer.io.getDpad() == 180) {
      IntakeSpeed = -1;
    }

    if (speaker && LaunchPermision() == 1 && sideSpeed > 0.5) {
      if (shotRecorder.get() < 0.0001) {
        ShotDistance.add(shotNumber,adjDistance);
        shotRecorder.start();
        shotNumber += 1;
      }
    } else {
      shotRecorder.stop();
      shotRecorder.reset();
    }
    
    io.setMotors(-ShootSpeed, -ShootSpeed, FeedSpeed, shooterAngle, IntakeSpeed, sideSpeed, limitOff, climber);//sends the run command to everything.
  }

  public double LaunchPermision() {//launch permision, identifies, when shot parameters are reached. (returns a number, as it was originaly inteded to serve as a controller vibration input.)
    if (shooterAngle < getAnlge().plus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && ShootSpeed < getAvrgShootSpd() + 40 && ShootSpeed > getAvrgShootSpd() - 40 && launchMode && autoAim && DriverStation.isTeleop()) {
      return 1;
    }else if (shooterAngle < getAnlge().plus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && shooterAngle > getAnlge().minus(new Rotation2d(Units.degreesToRadians(2))).getDegrees() && ShootSpeed < getAvrgShootSpd() + 40 && ShootSpeed > getAvrgShootSpd() - 40 && launchMode) {
      return 1;
    } else {
      return 0;
    }
  }

  public double IntakeRumble() {//Identify when the controller rumble should fire
    if (inputs.intakeLimit && IntakeSpeed > 0) {
      return 1;
    } else {
      return 0;
    }
  }

  public double lightRumble() {
    if(inputs.intakeAmps > 27) { //Identify when a note is being touched by the intake.
      return 0.25;
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
