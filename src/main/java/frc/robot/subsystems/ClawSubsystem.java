// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClawSubsystem extends SubsystemBase {
  
  DoubleSolenoid claw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);
  Value position = Value.kReverse;

  public ClawSubsystem() {}

  public void ClawClosed(){
    position = Value.kReverse;
  }

  public void ClawOpen(){
    position = Value.kForward;
  }

  @Override
  public void periodic() {
    claw.set(position);
  }

}
