// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IdentifierConstants;

public class Drive extends SubsystemBase {

  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;

  private final MotorControllerGroup left;
  private final MotorControllerGroup right;
  
  private final DifferentialDrive diffDrive;

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

    diffDrive = new DifferentialDrive(left, right);
    diffDrive.setSafetyEnabled(false);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double throttle, double rotate) {
    diffDrive.arcadeDrive(throttle, rotate);
  }

  public void zero() {
    left.set(0);
    right.set(0);
  }

  @Override
  public void periodic() {
  }
}
