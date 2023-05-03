// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;






import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Container.RobotContainer;
import frc.robot.Util.IO;
import frc.robot.Util.Constants.AR_SET;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Double m_autoSelected;
  boolean strafeToggle, stingerToggle, scoreMode, CollectMode, ConeMode, CubeMode, slowMode;
  private static final int PH_CAN_ID = 1;
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  private static final Timer autoTimer = new Timer();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {


   

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_ph.enableCompressorAnalog(90,115);
    SmartDashboard.putNumber("COmprressor", m_ph.getPressure(PH_CAN_ID));
    SmartDashboard.putNumber("AutoChoice", 0);
    SmartDashboard.putBoolean("resetEncoders", false);
    SmartDashboard.putNumber("ArmTemp", RobotContainer.ARM.getArmTemp());
    SmartDashboard.putNumber("ElbowTemp", RobotContainer.ARM.getWristTemp());
    SmartDashboard.putNumber("armENCODER", RobotContainer.ARM.getArmEncoder());
    SmartDashboard.putNumber("elbowENCODER", RobotContainer.ARM.getElbowEncoder());
    SmartDashboard.putNumber("leftDriveTemp", RobotContainer.DriveTrain.getLeftDriveTemp());
    SmartDashboard.putNumber("rightDriveTemp", RobotContainer.DriveTrain.getRightDriveTemp());
    SmartDashboard.putNumber("rightDriveEncoder", RobotContainer.DriveTrain.getRIGHTencoder());
    SmartDashboard.putNumber("leftEncoder", RobotContainer.DriveTrain.getLEFTencoder());
    SmartDashboard.putNumber("stingerEncoder", RobotContainer.DriveTrain.getSTINGERencoder());
    SmartDashboard.putNumber("strafeEncoder", RobotContainer.DriveTrain.getSTRAFEencoder());
    SmartDashboard.putNumber("leftEncoderSpeed", RobotContainer.DriveTrain.getLeftSpeed());
    SmartDashboard.putNumber("RightEncoderSpeed", RobotContainer.DriveTrain.getRightSpeed());
    SmartDashboard.putNumber("ArmCurrent", RobotContainer.ARM.getArmCurrent());
    SmartDashboard.putNumber("ElbowCurrent", RobotContainer.ARM.getElbowCurrent());
    if (SmartDashboard.getBoolean("resetEncoders", false) == true) {
      RobotContainer.DriveTrain.resetEncoders();
    }
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // resets drive encoders
   RobotContainer.DriveTrain.resetDriveEncoder();
   //resets and starts timers
   autoTimer.reset();
   autoTimer.start();
   //auto chooser Dont touch
    m_autoSelected = SmartDashboard.getNumber("AutoChoice", 0);
    // reset drive mode
    stingerToggle = false;
    strafeToggle = false;
    scoreMode = false;
    CollectMode = false;
    ConeMode = false;
    CubeMode = true;
    slowMode = false;
  
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    if(m_autoSelected == 1) { // 2 item
      // 6=1 foot Tank 10 opposite for 90 turn 15 for a foot strafe 23 strafe and 24.9 for strafe steer steer 90. 
        if(autoTimer.get()<2.5) { //two item auto movments
          RobotContainer.DriveTrain.autoStrafeDrive(-24, 24); // strafe over from first post
        } else if(autoTimer.get()>2.5 && autoTimer.get()<6.5) {
          RobotContainer.DriveTrain.autoTankDrive(-88, -88); //backup to seceond item
        }  else if( autoTimer.get()>6.6 && autoTimer.get()<6.7) {
          RobotContainer.DriveTrain.resetDriveEncoder(); //reset encoders
        } else if(autoTimer.get()>6.7 && autoTimer.get()<8.7) {
          RobotContainer.DriveTrain.autoTankDrive(20, -20); // turn 180
        } else if (autoTimer.get()>8.7 && autoTimer.get()<8.9) {
          RobotContainer.DriveTrain.resetDriveEncoder(); //turn 180
        } else if(autoTimer.get()>9 && autoTimer.get()<11.3) {
          RobotContainer.DriveTrain.autoTankDrive(21, -21); // turn 180 again
        } else if(autoTimer.get()>11.3 && autoTimer.get()<11.4) {
          RobotContainer.DriveTrain.resetDriveEncoder(); // reset drive
          RobotContainer.DriveTrain.autoTankDrive(0, 0); // stabalize drive
        } else if(autoTimer.get()>11.4 && autoTimer.get()<12.45) {
          RobotContainer.DriveTrain.runTankDrive(0, 0, true); // aim drivetrian
        } else if(autoTimer.get()>12.45 && autoTimer.get()<15) {
          RobotContainer.DriveTrain.autoTankDrive(92,92); //return to 2nd post
        } else {
          RobotContainer.DriveTrain.resetDriveEncoder(); // reset encoders
          RobotContainer.DriveTrain.autoTankDrive(0, 0); // ensure secore position
        }
        // Put custom auto code here
      }
      
      if(m_autoSelected == 0) {
      // 6=1 foot Tank 10 opposite for 90 turn 15 for a foot strafe 23 strafe and 24.9 opposite for strafe steer 90.
      if(autoTimer.get() <2.3) {
        RobotContainer.ARM.runArmSmartMotion(69, 0, false, false, 0, 0, false);
      }
      if(autoTimer.get() >2.3 && autoTimer.get()<3.3) {
        RobotContainer.ARM.runArmSmartMotion(69, 0, true, false, 0, 0, false);
      }
      if(autoTimer.get() >3.3 && autoTimer.get()<4.3) {
        RobotContainer.ARM.runArmSmartMotion(69, 0, true, false, -0.4, 0.4, false);
      }
      if(autoTimer.get() > 4.3 && autoTimer.get()< 5.6) {
        RobotContainer.ARM.runArmSmartMotion(0, 0, false, false, 0, 0, false);
      }
      if(autoTimer.get()>4.3&&autoTimer.get()<10.5) {
        RobotContainer.ARM.runArmSmartMotion(0, 0, false, false, 0, 0, false);
        RobotContainer.DriveTrain.autoTankDrive(-83, -83); // backup past line.
      } else if (autoTimer.get()>10.5 && autoTimer.get()<15){
        RobotContainer.ARM.runArmSmartMotion(0, 0, false, false, 0, 0, false);
        RobotContainer.DriveTrain.autoTankDrive(-38, -38); // pull forward and balance on ramp.
      }
        // Put default auto code here
      }
    }

    
  

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    RobotContainer.DriveTrain.resetDriveEncoder();
    stingerToggle = false;
    strafeToggle = false;
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("armPosition", 0);
    SmartDashboard.putNumber("elbowPosition", 0);
    SmartDashboard.putBoolean("manualArm", false);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
//DRIVE CONTROLLS

// toggle logic
if(IO.getLeftBumperPressed()) {
  strafeToggle = !strafeToggle;
  stingerToggle = false;
}
if(IO.getRightBumperPressed()) {
stingerToggle = !stingerToggle;
strafeToggle = false;
}
if(IO.getXbuttonPressed()) {
strafeToggle = false;
stingerToggle = false;
}


// driver train mode logic
  if (strafeToggle == true) {
    RobotContainer.DriveTrain.runStrafeDrive(IO.getLeftX(), IO.getRightX(), IO.getTriggerSteer());
  } else if (stingerToggle == true) {
    RobotContainer.DriveTrain.runStingerDrive(IO.getLeftY(), IO.getTriggerSteer(), IO.getRightX(), IO.getTriggerSteer());
  } else {
    RobotContainer.DriveTrain.runTankDrive(IO.getLeftY(), IO.getRightX(), IO.getBbutton());
  }

if (IO.getAbutton()) {
  RobotContainer.DriveTrain.resetDriveEncoder();
} else {}

//OPERATOR CONTROLLS
double setA, setE;
boolean setW, setL;
//slow mode logic
if (IO.getBbuttonPressed()){
  slowMode = !slowMode;
}
//mode toggle logic
if(IO.getLeftBumperPressedOp()) {
  scoreMode = !scoreMode;
  CollectMode = false;
}
if(IO.getRightBumperPressedOp()) {
  CollectMode = !CollectMode;
  scoreMode = false;
}
//claw togle logic
if(IO.getStartButtonPressedOp() == true) {
  ConeMode = !ConeMode;
  CubeMode = false;
}
if(IO.getRightTriggerButtonOp() == true) {
  CubeMode = !CubeMode;
  ConeMode = false;
}



if (scoreMode == true) {
  if(IO.getYbuttonOp()==true) {// high position 
    setA = AR_SET.HighPositionArm;
    setE = AR_SET.HighPositionElbow;
  } else if (IO.getBbuttonOp() == true) {//high position raised wrist
    setA = AR_SET.HighPositionArm;
    setE = 0;
  } else if (IO.getXButtonOp() == true) {//mid position
    setA = AR_SET.MidPositionArm;
    setE = AR_SET.MidPositionElbow;
  } else if (IO.getAButtonOp() == true) {//mid position raised wrist
    setA = AR_SET.MidPositionArm;
    setE = 0;
  } else if (IO.getPovOp()==270) {
    setA = AR_SET.homeArm;
    setE = AR_SET.homeElbow;
  } else {//floor position
    setA = AR_SET.floorArm;
    setE = AR_SET.floorElbow;
  }
} else if (CollectMode == true) {
  if(IO.getYbuttonOp() == true) {//cube/cone close
    setA = AR_SET.coneArm;
    setE = AR_SET.coneElbow;
  } else if (IO.getXButtonOp() == true) {//cube/cone reach
    setA = AR_SET.reachConeArm;
    setE = AR_SET.reachConeElbow;
  } else if (IO.getBbuttonOp() == true) {//station
    setA = AR_SET.stationArm;
    setE = AR_SET.stationElbow;
  } else if(IO.getAButtonOp() == true) {
    setA =AR_SET.stationArm;
    setE =0;
  }else if (IO.getPovOp()==270) {
    setA = AR_SET.homeArm;
    setE = AR_SET.homeElbow;
  } else {//stowed
    setA =  AR_SET.stowedArm;
    setE = AR_SET.stowedElbow;
  }
} else {
  setA =0;
  setE =0;
}
double clawMotor;
if (IO.getLeftTriggerSTG1Op()==true) {
  clawMotor = 0.5;
} else if (IO.getLeftTriggerSTG2Op()==true) {
  clawMotor = -0.35;
} else { 
  clawMotor = 0;
}
//light controls
if (IO.getPovOp()==90) {
  setL=false;
} else {
  setL = true;
}
if (IO.getLeftYUpButtonOp() == true) {
  setW = true;
} else {
  setW = false;
}
//claw selector
boolean clawMode;
if (ConeMode == true) {
  clawMode = true;
} else if (CubeMode == true ) {
  clawMode = false;
} else {
  clawMode = false;
}
//light selector
boolean cone,cube;
if (IO.getPovOp() == 0) {
  cone = true;
  cube = false;
} else if (IO.getPovOp() == 180) {
  cone = false;
  cube = true;
} else {
  cone = false;
  cube = false;
}
if(ConeMode) {
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
} else {
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
}
boolean home;
if (IO.getPovOp()==270) {
  home = true;
} else {
  home = false;
}
//light controller
RobotContainer.ARM.setLight(setL);
RobotContainer.ARM.setIndicators(cone, cube, CollectMode);
//arm controller
RobotContainer.ARM.runArmSmartMotion(setA, setE, setW, clawMode, clawMotor, -clawMotor, home);

SmartDashboard.putBoolean("Cone Mode",ConeMode);
SmartDashboard.putBoolean("ScoreMode", scoreMode);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
