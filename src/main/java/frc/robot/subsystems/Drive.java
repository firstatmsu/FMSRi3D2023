// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IdentifierConstants;


public class Drive extends SubsystemBase {

  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;

  public final DifferentialDriveKinematics kinematics;
  public final DifferentialDriveOdometry odometry;

  private final DifferentialDriveFeedforward feedforward;
  private final DifferentialDrive diffDrive;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private double timestamp;
 
  public Drive() {
    frontLeft = new CANSparkMax(IdentifierConstants.driveFL, MotorType.kBrushless);
    frontRight = new CANSparkMax(IdentifierConstants.driveFR, MotorType.kBrushless);
    backLeft = new CANSparkMax(IdentifierConstants.driveBL, MotorType.kBrushless);
    backRight = new CANSparkMax(IdentifierConstants.driveBR, MotorType.kBrushless);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    left = new MotorControllerGroup(frontLeft, backLeft);
    right = new MotorControllerGroup(frontRight, backRight);

    kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

    leftEncoder = frontLeft.getEncoder(Type.kQuadrature, 4096);
    rightEncoder = frontRight.getEncoder(Type.kQuadrature, 4096);

    leftEncoder.setPositionConversionFactor(DriveConstants.encoder);
    leftEncoder.setVelocityConversionFactor(DriveConstants.encoder);
    rightEncoder.setPositionConversionFactor(DriveConstants.encoder);
    rightEncoder.setVelocityConversionFactor(DriveConstants.encoder);
    
    diffDrive = new DifferentialDrive(left, right);
    diffDrive.setSafetyEnabled(false);

    feedforward = null;

    timestamp = Timer.getFPGATimestamp();
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  public void arcadeDrive(double leftVel, double rightVel) {
    double newTime = Timer.getFPGATimestamp();
    DifferentialDriveWheelVoltages voltages = feedforward.calculate(leftEncoder.getVelocity(), leftVel, rightEncoder.getVelocity(), rightVel, newTime-timestamp);
    timestamp = newTime;
    tankDrive(voltages.left, voltages.right);
  }

  public void zero() {
    left.set(0);
    right.set(0);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose.getRotation(), 0, 0, pose);
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(0), leftEncoder.getPosition(), rightEncoder.getPosition());
  }
}
