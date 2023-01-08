// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive;

public class TankDrive extends CommandBase {
  private final Drive drive;

  private final Supplier<Double> leftStick;
  private final Supplier<Double> rightStick;

  public TankDrive(Drive drive, Supplier<Double> leftStick, Supplier<Double> rightStick) {
    this.drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;

    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double leftSpeed = leftStick.get() * OperatorConstants.throttleMultiplier;
    double rightSpeed = rightStick.get() * OperatorConstants.throttleMultiplier;
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    drive.zero();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
