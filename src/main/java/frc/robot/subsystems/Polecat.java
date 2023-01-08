// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IdentifierConstants;

public class Polecat extends SubsystemBase {
  /** Creates a new Polecat. */
  protected final DigitalInput upperLimitSwitch;
  protected final DigitalInput lowerLimitSwitch;

  private final PWMSparkMax poleCat;

  public Polecat() {
    poleCat = new PWMSparkMax(IdentifierConstants.polecat);
    upperLimitSwitch = new DigitalInput(IdentifierConstants.polecatUpperSwitch);
    lowerLimitSwitch = new DigitalInput(IdentifierConstants.polecatLowerSwitch);
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
