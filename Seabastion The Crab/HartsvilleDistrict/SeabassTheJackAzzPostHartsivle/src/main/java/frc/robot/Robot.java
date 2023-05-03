// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Container.RobotContainer;
import frc.robot.ControllsProcessing.ClawControlls;
import frc.robot.ControllsProcessing.DriveTrain.StrafeControlls;
import frc.robot.ControllsProcessing.DriveTrain.TankControlls;
import frc.robot.Util.IO;
import frc.robot.Util.Constants.DT_Set;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kRamp = "ramp";
  private static final String ARamp = "autoRamp";
  private static final int PH_CAN_ID = 1;
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  PneumaticHub m_ph = new PneumaticHub(PH_CAN_ID);
  private final Timer autoTimer = new Timer();

//toggles
  boolean coneMode, CubeMode, Tank, strafe, stinger;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("ramp", kRamp);
    m_chooser.addOption("autoRamp", ARamp);
    SmartDashboard.putData("Auto choices", m_chooser);
    coneMode = false;
    CubeMode = true;
    Tank = true;
    strafe = false;
    stinger = false;
    autoTimer.reset();
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
    SmartDashboard.putBoolean("resetEncoders", false);
    SmartDashboard.putNumber("armENCODER", RobotContainer.ARM.getArmEncoder());
    SmartDashboard.putNumber("elbowENCODER", RobotContainer.ARM.getElbowEnocoder());
    SmartDashboard.putNumber("leftDriveTemp", RobotContainer.DriveTrain.getLeftDriveTemp());
    SmartDashboard.putNumber("rightDriveTemp", RobotContainer.DriveTrain.getRightDriveTemp());
    SmartDashboard.putNumber("rightDriveEncoder", RobotContainer.DriveTrain.getRightDriveEncoder());
    SmartDashboard.putNumber("leftEncoder", RobotContainer.DriveTrain.getLeftDriveEncoder());
    SmartDashboard.putNumber("stingerEncoder", RobotContainer.DriveTrain.getStingerEncoder());
    SmartDashboard.putNumber("strafeEncoder", RobotContainer.DriveTrain.getStrafeEncoder());
    SmartDashboard.putNumber("leftEncoderSpeed", RobotContainer.DriveTrain.getLeftDriveSpeed());
    SmartDashboard.putNumber("RightEncoderSpeed", RobotContainer.DriveTrain.getRightDriveSpeed());
    SmartDashboard.putNumber("Arm Current", RobotContainer.ARM.getArmCurrent());
    SmartDashboard.putNumber("elbow current", RobotContainer.ARM.getElbowCurrent());
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
    m_autoSelected = m_chooser.getSelected();
    autoTimer.reset();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoTimer.start();
    switch (m_autoSelected) {
      case kRamp:
        if (autoTimer.get() < 1.3 ) {
          RobotContainer.DriveTrain.zeroDrive();
          RobotContainer.DriveTrain.runTankAuto(0, 0, false);
          RobotContainer.ARM.runArmSmartMotion(6, 0, false);
          RobotContainer.CLAW.runClaw(0, 0, true, false, false, false);
        } else if (autoTimer.get() < 2.5) {
          RobotContainer.DriveTrain.runTankAuto(0, 0, false);
          RobotContainer.ARM.runArmSmartMotion(6, 0, false);
          RobotContainer.CLAW.runClaw(-1, 1, true, false, false, false);
        } else if (autoTimer.get() < 7.2) {
          RobotContainer.DriveTrain.runTankAuto(-78, 78, false);
          RobotContainer.ARM.runArmSmartMotion(0, 0, false);
          RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
        } else if (autoTimer.get() < 11) {
          RobotContainer.DriveTrain.runTankAuto(-39, 39, false);
          RobotContainer.ARM.runArmSmartMotion(0, 0, false);
          RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
        } else {
          RobotContainer.DriveTrain.runTankAuto(-39, -39, true);
          RobotContainer.ARM.runArmSmartMotion(0, 0, false);
          RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
        } 
        // Put custom auto code here
        break;
      case ARamp:
      if (autoTimer.get() < 1.3 ) {
        RobotContainer.DriveTrain.zeroDrive();
        RobotContainer.DriveTrain.runTankAuto(0, 0, false);
        RobotContainer.ARM.runArmSmartMotion(6, 0, false);
        RobotContainer.CLAW.runClaw(0, 0, true, false, false, false);
      } else if (autoTimer.get() < 2.5) {
        RobotContainer.DriveTrain.runTankAuto(0, 0, false);
        RobotContainer.ARM.runArmSmartMotion(6, 0, false);
        RobotContainer.CLAW.runClaw(-1, 1, true, false, false, false);
      } else if (autoTimer.get() < 7.2) {
        RobotContainer.DriveTrain.runTankAuto(-78, 78, false);
        RobotContainer.ARM.runArmSmartMotion(0, 0, false);
        RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
      } else if (autoTimer.get() < 11) {
        RobotContainer.DriveTrain.runTankAuto(-78, 78, false);
        RobotContainer.ARM.runArmSmartMotion(0, 0, false);
        RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
      } else {
        RobotContainer.DriveTrain.runTankAuto(-78, 78, false);
        RobotContainer.ARM.runArmSmartMotion(0, 0, false);
        RobotContainer.CLAW.runClaw(0, 0, false, false, false, false);
      } 
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

      } else if (IO.getAbutton() == true) {
        ArmPosition1 = 5;
        elbowPosition1 = 8;

      } else if(IO.getBbuttonOp() == true) {
        ArmPosition1 = 52;
        elbowPosition1 = 13;

      } else if (Home == true) {
        ArmPosition1 = -40;
        elbowPosition1 = -20;

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
  
      //Drive Controlls
      //toggles
      if(IO.getLeftBumperPressed()) {
        strafe = !strafe;
        stinger = false;
      }
      if (IO.getRightBumperPressed()) {
        stinger = !stinger;
        strafe = false;
      }
      //controlls
      double left2, right2, stinger2, strafe2, Tacc, StAcc, Sacc;//tacc = tank accel, StAcc = stinger accel, Sacc = strafe accel;
      boolean strafebutter, stingerbutter;
  
      double forward = IO.getLeftY()*0.6;
      double sideways = IO.getLeftX() * 0.2;
      double rearSideways = IO.getLeftX() * 1;
      double rotate =  sideways + IO.getRightX()*0.8;
      double rearSteer = rearSideways + IO.getTriggerSteer();
      //stinger Controlls
     
      //Mode Controlls
      if (stinger == true) {
        double sens = DT_Set.DT_TURN_SENSITIVITY;
            if (forward == 0) {
                sens = DT_Set.DT_QUICK_TURN;
            }
  
            double left = -forward + rotate * sens;
            double right = -forward - rotate * sens;
            double stinger = rearSteer *sens;
        double maxMag = Math.abs(stinger);
            if (maxMag > 1) {
              stinger = 1;
            } else {
              
            }
        left2 = left;
        right2 = right;
        stinger2 = -stinger;
        strafe2 = 0;
        strafebutter = false;
        stingerbutter = true;
        Tacc = 0.2;
        StAcc = 0;
        Sacc = 0;
      }else if (strafe == true) {
        left2 = 0;
        right2 = 0;
        stinger2 = StrafeControlls.StingerS(IO.getLeftX(), IO.getRightX());
        strafe2 = StrafeControlls.StrafeS(IO.getLeftX(), IO.getRightX());
        strafebutter = true;
        stingerbutter = true;
        Tacc = 0;
        StAcc = 3;
        Sacc = 3;
      } else {
        left2 = TankControlls.LeftT(IO.getLeftY(), IO.getRightX());
        right2 = TankControlls.RightT(IO.getLeftY(), IO.getRightX());
        stinger2 = 0;
        strafe2 = 0;
        strafebutter = false;
        stingerbutter = false;
        Tacc = 0.8;
        StAcc = 0;
        Sacc = 0;
      }
      //max setters
      double maxMagnitude = Math.max(Math.abs(left2), Math.abs(right2));
      if (maxMagnitude > 1){
          left2 /= maxMagnitude;
          right2 /= maxMagnitude;
      }
      double maxMagn = Math.max(Math.abs(stinger2), Math.abs(strafe2));
      if (maxMagnitude > 1){
          stinger2 /= maxMagn;
          strafe2 /= maxMagn;
      }
      //drivetrain setter.
      RobotContainer.DriveTrain.runDriveTrain(left2, right2, strafe2, stinger2, strafebutter, stingerbutter, Tacc, Sacc, StAcc,IO.getYbutton(), IO.getLeftY(), IO.getLeftX(), IO.getBbutton());

      if(coneMode) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      } else {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
