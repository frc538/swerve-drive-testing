/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class SwerveModule {
  private final double STEERING_RATIO = 18;
  private final double RADIANS_PER_ROTATION = 2 * Math.PI;
  private final double SECONDS_PER_MINUTE = 60;
  private final double RADIUS_METERS = 2 * 2.54 / 100;
  private final double STAGE_ONE_DRIVER = 42;
  private final double STAGE_ONE_DRIVEN = 14;
  private final double STAGE_TWO_DRIVER = 18;
  private final double STAGE_TWO_DRIVEN = 26;
  private final double STAGE_THREE_DRIVER = 60;
  private final double STAGE_THREE_DRIVEN = 15;
  private final double DRIVING_RATIO = (STAGE_ONE_DRIVER / STAGE_ONE_DRIVEN) * (STAGE_TWO_DRIVER / STAGE_TWO_DRIVEN)
      * (STAGE_THREE_DRIVER / STAGE_THREE_DRIVEN);

  private final CANSparkMax drive;
  private final CANSparkMax turn;
  private final CANEncoder driveEncoder;
  private final CANEncoder turnEncoder;
  private final CANPIDController driveController;
  private final CANPIDController turnController;
  private final AnalogEncoder absEncoder;
  private final Rotation2d mAngleOffset;

  private final int TURN_ID;

  public SwerveModule(int driveId, int turnId, int absEncId, Rotation2d angleOffset) {
    TURN_ID = turnId;
    mAngleOffset = angleOffset;

    drive = new CANSparkMax(driveId, MotorType.kBrushless);
    turn = new CANSparkMax(turnId, MotorType.kBrushless);

    drive.restoreFactoryDefaults();
    turn.restoreFactoryDefaults();

    driveEncoder = drive.getEncoder();
    turnEncoder = turn.getEncoder();

    driveEncoder.setVelocityConversionFactor(RADIANS_PER_ROTATION / SECONDS_PER_MINUTE * RADIUS_METERS / DRIVING_RATIO);
    turnEncoder.setPositionConversionFactor(RADIANS_PER_ROTATION / STEERING_RATIO);

    absEncoder = new AnalogEncoder(new AnalogInput(absEncId));
    absEncoder.setDistancePerRotation(RADIANS_PER_ROTATION);

    initializeTurnEncoder();

    driveController = new CANPIDController(drive);
    turnController = new CANPIDController(turn);

    driveController.setP(0.0001);
    driveController.setFF(0.000171);

    turnController.setP(0.08);
  }

  private double getScaledAbsoluteEncoderReading() {
    return absEncoder == null ? 0 : absEncoder.getDistance() % RADIANS_PER_ROTATION;
  }

  public void initializeTurnEncoder() {
    turnEncoder.setPosition(getScaledAbsoluteEncoderReading() - mAngleOffset.getRadians());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
  }

  public void setState(SwerveModuleState state) {
    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }

  public void putData() {
    SmartDashboard.putNumber("Absolute Encoder " + TURN_ID + ":", getScaledAbsoluteEncoderReading() * 180 / Math.PI);
    SmartDashboard.putNumber("Turn Encoder" + TURN_ID + ":", turnEncoder.getPosition() * 180 / Math.PI);
  }
}