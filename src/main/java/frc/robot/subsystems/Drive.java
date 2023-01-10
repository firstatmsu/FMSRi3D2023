// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IdentifierConstants;


public class Drive extends SubsystemBase {

  private final PWMSparkMax frontLeft;
  private final PWMSparkMax frontRight;
  private final PWMSparkMax backLeft;
  private final PWMSparkMax backRight;

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;

  private final DifferentialDrive diffDrive;

  public Drive() {
    frontLeft = new PWMSparkMax(IdentifierConstants.driveFL);
    frontRight = new PWMSparkMax(IdentifierConstants.driveFR);
    backLeft = new PWMSparkMax(IdentifierConstants.driveBL);
    backRight = new PWMSparkMax(IdentifierConstants.driveBR);

    // backLeft.setInverted(true);
    // backRight.setInverted(true);

    left = new MotorControllerGroup(frontLeft, backLeft);
    right = new MotorControllerGroup(frontRight, backRight);
    left.setInverted(true);

    diffDrive = new DifferentialDrive(left, right);
    diffDrive.setSafetyEnabled(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
  }

  public void arcadeDrive(double throttleSpeed, double rotateSpeed) {
    diffDrive.arcadeDrive(throttleSpeed, rotateSpeed);
  }

  public void setAll() {
    frontLeft.set(0.5);
    frontRight.set(0.5);
    backLeft.set(0.5);
    backRight.set(0.5);
  }

  public void zero() {
    left.set(0);
    right.set(0);
  }

}
