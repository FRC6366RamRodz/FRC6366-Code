// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveNEO implements DriveIO {
  private static final double gearRatio =
      13.0; // roughly 10ft a second with six inch wheels and dual neo per side

  private final CANSparkMax LEFT_FRONT =
      new CANSparkMax(
          1,
          MotorType
              .kBrushless); // identify device type, set a call name, offer an ID # for that motor
  // and motor type.
  private final CANSparkMax RIGHT_FRONT = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax LEFT_REAR = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax RIGHT_REAR = new CANSparkMax(4, MotorType.kBrushless);

  private final RelativeEncoder LEFT_ENCODER = LEFT_FRONT.getEncoder();
  private final RelativeEncoder RIGHT_ENCODER = RIGHT_FRONT.getEncoder();

  // Optional pigeon gyro
  // private final Pigeon2 pigeon = new Pigeon2(20);

  public DriveNEO() {
    LEFT_FRONT.restoreFactoryDefaults(false); // restore factory defaults to ensure a clean slate
    RIGHT_FRONT.restoreFactoryDefaults(false);
    LEFT_REAR.restoreFactoryDefaults(false);
    RIGHT_REAR.restoreFactoryDefaults(false);

    LEFT_FRONT.setCANTimeout(250); // fixes can timeout error when enabling.
    RIGHT_FRONT.setCANTimeout(250);
    LEFT_REAR.setCANTimeout(250);
    RIGHT_FRONT.setCANTimeout(250);

    LEFT_FRONT.setInverted(false); // set inverts and flowing
    RIGHT_FRONT.setInverted(false);
    LEFT_REAR.follow(LEFT_FRONT, false);
    RIGHT_REAR.follow(RIGHT_FRONT, false);

    LEFT_FRONT.enableVoltageCompensation(
        12.0); // minimizes the negative affect of running percent out and may make it superior to
    // encoder drive for handling.
    RIGHT_FRONT.enableVoltageCompensation(12.0);
    LEFT_FRONT.setSmartCurrentLimit(40); // set peak allowed current in amps
    RIGHT_FRONT.setSmartCurrentLimit(40);

    LEFT_FRONT.burnFlash(); // saves setting to hardware memory
    RIGHT_FRONT.burnFlash();
    LEFT_REAR.burnFlash();
    RIGHT_REAR.burnFlash();

    /*optional pigeon stuff
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        pigeon.optimizeBusUtilization();
    */
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) { // logged values
    inputs.leftPositionRad = Units.rotationsToDegrees(LEFT_ENCODER.getPosition() / gearRatio);
    inputs.rightPositionRad = Units.rotationsToRadians(RIGHT_ENCODER.getPosition() / gearRatio);

    inputs.leftAvgVolts = (LEFT_FRONT.getAppliedOutput() + LEFT_REAR.getAppliedOutput()) / 2;
    inputs.rightAvgVolts = (RIGHT_FRONT.getAppliedOutput() + RIGHT_REAR.getAppliedOutput()) / 2;

    inputs.leftVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(
            LEFT_ENCODER.getVelocity()
                / gearRatio); // divide by gear ratio to get wheel speed or dont to get motor speed.
    inputs.rightVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(RIGHT_ENCODER.getVelocity() / gearRatio);

    inputs.leftAvgAmps = (LEFT_FRONT.getOutputCurrent() + LEFT_REAR.getOutputCurrent()) / 2;
    inputs.rightAvgAmps = (RIGHT_FRONT.getOutputCurrent() + RIGHT_REAR.getOutputCurrent()) / 2;

    inputs.leftAvgTemp = (LEFT_FRONT.getMotorTemperature() + LEFT_REAR.getMotorTemperature()) / 2;
    inputs.rightAvgTemp =
        (RIGHT_FRONT.getMotorTemperature() + RIGHT_REAR.getMotorTemperature()) / 2;

    /*optional pigeon gyro
     * inputs.gyroYaw = Rotation2d.fromDegrees(pigeon.getYaw);
     */
    // odometry workaround may need the signs swapped
    inputs.gyroYaw = new Rotation2d(inputs.leftPositionRad - inputs.rightPositionRad);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) { // set values
    LEFT_FRONT.setVoltage(leftVolts);
    RIGHT_FRONT.setVoltage(rightVolts);
  }
}
