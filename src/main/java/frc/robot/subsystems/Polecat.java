// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IdentifierConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class Polecat extends SubsystemBase {
  /** Creates a new Polecat. */
  private final CANSparkMax poleCat;
  protected final DigitalInput upperLimitSwitch;
  protected final DigitalInput lowerLimitSwitch;

  public Polecat() {
    poleCat = new CANSparkMax(IdentifierConstants.polecat, MotorType.kBrushless);
    upperLimitSwitch = new DigitalInput(IdentifierConstants.polecatUpperSwitch);
    lowerLimitSwitch = new DigitalInput(IdentifierConstants.polecatLowerSwitch);
  }

  public enum Direction {
    kUp,
    kDown,
  }

  public void movePoleCat() {
    poleCat.set(OperatorConstants.poleCatSpeed);
  }

  public boolean isLimited(Direction direction) {
    return (direction == Direction.kUp) ? upperLimitSwitch.get() : lowerLimitSwitch.get();
  }

  public void stopPoleCat() {
    poleCat.stopMotor();
  }

}
