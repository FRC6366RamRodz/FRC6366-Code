// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.AutoStuff.AutoAim;
import frc.robot.commands.AutoStuff.AutoLineShot;
import frc.robot.commands.AutoStuff.Intake;
import frc.robot.commands.AutoStuff.StageShot;
import frc.robot.commands.AutoStuff.WingShot;
import frc.robot.commands.AutoStuff.doNothing;
import frc.robot.commands.AutoStuff.shoot;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterSim;
import frc.robot.subsystems.Shooter.ShooterV3Hardware;
import frc.robot.subsystems.Vision.MultiCameraContainer;
import frc.robot.subsystems.Vision.SoloCameraContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.IO;
import frc.robot.util.LocalADStarAK;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;
  public static Shooter shooter;
  public static IO io = new IO();
  public static LocalADStarAK localADstar = new LocalADStarAK();
  public static AprilTagFieldLayout aprilTag = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public static SoloCameraContainer FrontLeftcam = new SoloCameraContainer("FrontLeft", Constants.frontLeftCamera, aprilTag);
  public static SoloCameraContainer FrontRightcam = new SoloCameraContainer("FrontRight", Constants.frontRightCamera, aprilTag);
  public static SoloCameraContainer BackRightcam = new SoloCameraContainer("BackRight", Constants.BackRightCamera, aprilTag);
  public static SoloCameraContainer BackLeftcam = new SoloCameraContainer("BackLeft", Constants.BackLeftCamera, aprilTag);
  public static MultiCameraContainer cameras = new MultiCameraContainer(FrontLeftcam,FrontRightcam,BackRightcam,BackLeftcam); //create all solo cameras, then name them in the multi cam
  public Command Intake = new Intake();
  public Command shoot = new shoot();
  public Command autoLineShot = new AutoLineShot();
  public Command wingShot = new WingShot();
  public Command doNothing = new doNothing();
  public Command stageShot = new StageShot();
  public Command autoAim = new AutoAim(); 
  
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);//for command stuff.

  // Dashboard inputs
  public final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        shooter = new Shooter(new ShooterV3Hardware());
         //drive =
         //   new Drive(
         //       new GyroIOPigeon2(),
         //       new ModuleIOSparkMax(0),
         //       new ModuleIOSparkMax(1),
         //       new ModuleIOSparkMax(2),
         //       new ModuleIOSparkMax(3));
        drive =
           new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

      case SIM:
        shooter = new Shooter(new ShooterSim());
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        shooter = new Shooter(new ShooterIO() {});
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    NamedCommands.registerCommand("intake", Intake);//create command for path planner to see.
    NamedCommands.registerCommand("shoot", shoot);
    NamedCommands.registerCommand("autoLineShot", autoLineShot);
    NamedCommands.registerCommand("WingShot", wingShot);
    NamedCommands.registerCommand("doNothing", doNothing);
    NamedCommands.registerCommand("StageShot", stageShot);
    NamedCommands.registerCommand("AutoAim", autoAim);
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); // create choosable dash value.

    // Set up feedforward characterization
    autoChooser.addOption("Drive FF Characterization", new FeedForwardCharacterization(drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity)); //add custom options.
    autoChooser.addOption("Wheel Radius Calibration",new WheelRadiusCharacterization(drive));

    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand( DriveCommands.joystickDrive( drive, () -> (-controller.getLeftY()), () -> (-controller.getLeftX()), () -> -controller.getRightX(), controller.rightBumper(), controller.leftBumper()));
    
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    
    controller.b().onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())), drive).ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
