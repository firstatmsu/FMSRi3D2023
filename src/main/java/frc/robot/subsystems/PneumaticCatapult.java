// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.IdentifierConstants;

public class PneumaticCatapult extends SubsystemBase {
  /** Creates a new PneumaticCatapult. */
  private final DoubleSolenoid pneumaticCatapult;
  public PneumaticCatapult() {
    pneumaticCatapult = new DoubleSolenoid(PneumaticsModuleType.REVPH, IdentifierConstants.solenoidForward, IdentifierConstants.solenoidReverse);
  }

  public void launch() {
    pneumaticCatapult.set(DoubleSolenoid.Value.kForward);
  }

  public void retract() {
    pneumaticCatapult.set(DoubleSolenoid.Value.kReverse);
  }
}