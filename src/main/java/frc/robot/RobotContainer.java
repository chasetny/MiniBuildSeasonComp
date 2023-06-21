// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverJoystick = new Joystick(0);
  
  public RobotContainer(){
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> driverJoystick.getRawAxis(1), 
      () -> driverJoystick.getRawAxis(0), 
      () -> driverJoystick.getRawAxis(4)
    ));

    
    configureButtonBindings();
  }

  public void configureButtonBindings(){
  }

  public Command getAutonomousCommand(){
    return null;
  }
}
