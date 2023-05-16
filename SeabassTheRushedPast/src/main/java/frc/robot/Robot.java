// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Container.RobotContainer;
import frc.robot.ControllsProcessing.ClawControlls;
import frc.robot.Util.IO;
import frc.robot.Util.Constants.DT_STG;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //toggles
    boolean STRAFE, STINGER; //toggle varriable remember their last state and maintain that state unless changed, shoule be all caps no _
    
  boolean coneMode, CubeMode, Tank, strafe, stinger;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     //variables
     double ArmPosition1, elbowPosition1, elbowPosition2, ArmPositionA; //1 = clean input //2 = 1 stage of proccesing //3 = this controll is overcomplicated
     boolean Home;
     ArmPositionA = RobotContainer.ARM.getArmEncoder();
     //Toggles
     if(IO.getLeftBumperPressedOp()) {
       coneMode = !coneMode;
       CubeMode = false;
     }
     if (IO.getRightBumperPressedOp()) {
       CubeMode = !CubeMode;
       coneMode = false;
     }
     //extras
     if (IO.getPovOp() == 270) {
       Home = true;
     } else {
       Home = false;
     }
     if (IO.getAButtonOp() == true) {
       ArmPosition1 = 26;
       elbowPosition1 = 0;

     } else if (IO.getXButtonOp()==true) {
       ArmPosition1 = 45;
       elbowPosition1 = 4;

     } else if (IO.getXButton() == true) {
       ArmPosition1 = 5;
       elbowPosition1 = 8;

     } else if(IO.getBbuttonOp() == true) {
       ArmPosition1 = 52.5;
       elbowPosition1 = 13;

     } else if (Home == true) {
       ArmPosition1 = -30;
       elbowPosition1 = -10;

     } else {
     ArmPosition1 = 0;
     elbowPosition1 = 0;

     }

     if (ArmPositionA > ArmPosition1-1.4) {
       elbowPosition2 = elbowPosition1;
     } else {
       elbowPosition2 = 0;
     }
     //set controlls
     RobotContainer.ARM.runArmSmartMotion(ArmPosition1, elbowPosition2,Home);
 
     //claw Controlls
     //variables
     double clawMotor;
     boolean Cone, Cube, clawMode;
     //extra 
     if (IO.getPovOp() == 0) {
       Cone = true;
     } else {
       Cone = false;
     }
     if (IO.getPOVButton() == 180) {
       Cube = true;
     } else {
       Cube = false;
     }
     if (coneMode == true) {
       clawMode = true;
     } else if (CubeMode == true) {
       clawMode = false;
     } else {
       clawMode = false;
     }
     clawMotor = ClawControlls.clawMotor(IO.getLeftTriggerSTG1Op(), IO.getLeftTriggerSTG2Op(), coneMode);
     RobotContainer.CLAW.runClaw(clawMotor, -clawMotor, IO.getLeftYUpButtonOp(), Cone, Cube, clawMode);
 
    //drive controlls 
    double leftMotor, rightMotor, frontDropMotor, rearDropMotor, forward, sideways, rotate, sens, fward, acro, maxMagleft, maxMagRight,maxMagFront, maxMagRear;
    boolean frontDrop, rearDrop;

    if (IO.getLeftBumperPressed()) {
      STRAFE = !STRAFE;
      STINGER = false;
    }
    if (IO.getRightBumperPressed()) {
      STRAFE = false;
      STINGER = !STINGER;
    }
    
    forward = IO.getLeftY();
    sideways = IO.getLeftX();
    rotate = IO.getRightX();

    if (forward == 0) {
      sens = 0.8;
    }else {
    sens = DT_STG.Drv_Sens;
    }

    fward = Math.abs(forward);

    if (IO.getLeftTrigger()) {
      acro = 1;
    } else {
      acro = 0.2;
    }

    if (STINGER) {

      leftMotor = -forward*acro + (rotate+(sideways*0.2)) * sens;
      rightMotor = -forward*acro - (rotate+(sideways*0.2)) * sens;
      rearDropMotor = (rotate+(sideways))*sens;
      frontDropMotor = 0;

      frontDrop = false;
      rearDrop = true;

    } else if (STRAFE) {

      leftMotor = 0;
      rightMotor = 0;
      rearDropMotor = sideways + rotate * fward * sens;
      frontDropMotor = sideways - rotate * fward *sens;

      frontDrop = true;
      rearDrop = true;

    } else {

      leftMotor = forward + rotate * fward *sens;
      rightMotor = forward - rotate * fward * sens;
      rearDropMotor = 0;
      frontDropMotor = 0;

      frontDrop = false;
      rearDrop = false;
    }

    maxMagleft = Math.abs(leftMotor);
    maxMagRight = Math.abs(rightMotor);
    maxMagFront = Math.abs(frontDropMotor);
    maxMagRear = Math.abs(rearDropMotor);

    if (maxMagleft > 1) {
      leftMotor /= maxMagleft;
    }
    if (maxMagRight > 1) {
      rightMotor /= maxMagRight;
    }
    if (maxMagFront > 1) {
      frontDropMotor /= maxMagFront;
    }
    if (maxMagRear > 1) {
      rearDropMotor /= maxMagRear;
    }

    RobotContainer.DriveTrain.runDriveTrain(IO.getLeftY(), IO.getLeftX(), leftMotor, rightMotor, frontDropMotor, rearDropMotor, frontDrop, rearDrop, IO.getYButton(), IO.getLeftTrigger());

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
