// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IdentifierConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.RelativeEncoder;

public class Polecat extends SubsystemBase {
  /** Creates a new Polecat. */
  private final CANSparkMax poleCat;
  protected final DigitalInput upperLimitSwitch;
  protected final DigitalInput lowerLimitSwitch;

  protected final RelativeEncoder poleCatEncoder;

  public Polecat() {
    poleCat = new CANSparkMax(IdentifierConstants.polecat, MotorType.kBrushless);
    upperLimitSwitch = new DigitalInput(IdentifierConstants.polecatUpperSwitch);
    lowerLimitSwitch = new DigitalInput(IdentifierConstants.polecatLowerSwitch);
    poleCatEncoder = poleCat.getEncoder(Type.kQuadrature, 4096);
    poleCatEncoder.setPositionConversionFactor(DriveConstants.encoder);
    poleCatEncoder.setVelocityConversionFactor(DriveConstants.encoder);
  }

  public enum Direction {
    kUp,
    kDown,
  }

  public void movePoleCat(double speed) {
    poleCat.set(speed);
  }

  public boolean isLimited(Direction direction) {
    return (direction == Direction.kUp) ? upperLimitSwitch.get() : lowerLimitSwitch.get();
  }

  public void stopPoleCat() {
    poleCat.stopMotor();
  }

}
