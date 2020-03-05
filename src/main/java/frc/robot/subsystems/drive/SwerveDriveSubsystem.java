/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
  private final double INCHES_TO_METERS = 2.54 / 100;
  private final double MAX_LINEAR_SPEED = 3.62712;

  private final double shooterX = -9.248 * INCHES_TO_METERS;

  private final Translation2d frontLeftLocation = new Translation2d(-4.405 * INCHES_TO_METERS - shooterX,
      -10 * INCHES_TO_METERS);
  private final Translation2d frontRightLocation = new Translation2d(-4.405 * INCHES_TO_METERS - shooterX,
      10 * INCHES_TO_METERS);
  private final Translation2d rearLeftLocation = new Translation2d(-27.905 * INCHES_TO_METERS - shooterX,
      -10 * INCHES_TO_METERS);
  private final Translation2d rearRightLocation = new Translation2d(-27.905 * INCHES_TO_METERS - shooterX,
      10 * INCHES_TO_METERS);

  private final SwerveModule frontLeftModule = new SwerveModule(Constants.CAN_ID.FRONT_LEFT_DRIVE,
      Constants.CAN_ID.FRONT_LEFT_TURN, Constants.FRONT_LEFT_ABS_ENCODER_ID, new Rotation2d(1.464952));
  private final SwerveModule frontRightModule = new SwerveModule(Constants.CAN_ID.FRONT_RIGHT_DRIVE,
      Constants.CAN_ID.FRONT_RIGHT_TURN, Constants.FRONT_RIGHT_ABS_ENCODER_ID, new Rotation2d(7.976699));
  private final SwerveModule rearLeftModule = new SwerveModule(Constants.CAN_ID.REAR_LEFT_DRIVE,
      Constants.CAN_ID.REAR_LEFT_TURN, Constants.REAR_LEFT_ABS_ENCODER_ID, new Rotation2d(3.198350));
  private final SwerveModule rearRightModule = new SwerveModule(Constants.CAN_ID.REAR_RIGHT_DRIVE,
      Constants.CAN_ID.REAR_RIGHT_TURN, Constants.REAR_RIGHT_ABS_ENCODER_ID, new Rotation2d(4.532913));

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation,
      rearLeftLocation, rearRightLocation);

  private boolean testing = false;

  /**
   * Creates a new SwerveDriveSubsystem.
   */
  public SwerveDriveSubsystem() {

  }

  public void initialize() {
    frontLeftModule.initializeTurnEncoder();
    rearLeftModule.initializeTurnEncoder();
    frontRightModule.initializeTurnEncoder();
    rearRightModule.initializeTurnEncoder();
  }

  private double linearSpeed(double unitRate) {
    return Math.pow(unitRate, 3.0) * MAX_LINEAR_SPEED;
  }

  private double angularSpeed(double unitRate) {
    return Math.pow(unitRate, 3.0) * Math.PI;
  }

  public void drive(double forwardRate, double rightRate, double rotationRate) {
    SmartDashboard.putNumber("Forward", forwardRate);
    SmartDashboard.putNumber("Right", rightRate);
    SmartDashboard.putNumber("Rotation", rotationRate);


    SwerveModuleState[] states = kinematics.toSwerveModuleStates(
        new ChassisSpeeds(linearSpeed(forwardRate), linearSpeed(rightRate), angularSpeed(rotationRate)));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, MAX_LINEAR_SPEED);

    frontLeftModule.setState(states[0]);
    frontRightModule.setState(states[1]);
    rearLeftModule.setState(states[2]);
    rearRightModule.setState(states[3]);
  }

  public void toggleTesting() {
    testing = !testing;
  }

  public void init() {
    frontLeftModule.initializeTurnEncoder();
    frontRightModule.initializeTurnEncoder();
    rearLeftModule.initializeTurnEncoder();
    rearRightModule.initializeTurnEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
