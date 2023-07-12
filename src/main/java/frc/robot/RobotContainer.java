// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmHigh;
import frc.robot.commands.ArmMid;
import frc.robot.commands.ArmZeroPosition;
import frc.robot.commands.ClawClosed;
import frc.robot.commands.ClawOpen;
import frc.robot.commands.ElevatorHighPosition;
import frc.robot.commands.ElevatorLowPosition;
import frc.robot.commands.ElevatorMidPosition;
import frc.robot.commands.ElevatorZeroPosition;
import frc.robot.commands.IntakeZero;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveStop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  // private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final Joystick driverJoystick = new Joystick(0);
  private final Joystick operJoystick = new Joystick(1);
  private final XboxController xbox = new XboxController(0);
  
  public RobotContainer(){
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem, 
      () -> driverJoystick.getRawAxis(1), 
      () -> driverJoystick.getRawAxis(0), 
      () -> driverJoystick.getRawAxis(4)
    ));

    //intakeSubsystem.setDefaultCommand(new IntakeZero(intakeSubsystem));
    configureButtonBindings();
  }

  public void configureButtonBindings(){

    // new JoystickButton(xbox, 1).onTrue(new ElevatorZeroPosition(elevatorSubsystem));
    // new JoystickButton(xbox, 2).onTrue(new ElevatorLowPosition(elevatorSubsystem));
    // new JoystickButton(xbox, 3).onTrue(new ElevatorMidPosition(elevatorSubsystem));
    // new JoystickButton(xbox, 4).onTrue(new ElevatorHighPosition(elevatorSubsystem));

    // new JoystickButton(xbox, 5).whileTrue(new IntakeForward(intakeSubsystem));
    // new JoystickButton(xbox, 6).whileTrue(new IntakeReverse(intakeSubsystem));

    // new JoystickButton(xbox, 7).onTrue(new ClawClosed(clawSubsystem));
    // new JoystickButton(xbox, 8).onTrue(new ClawOpen(clawSubsystem));

    // new JoystickButton(operJoystick, 1).onTrue(new ArmMid(armSubsystem));
    // new JoystickButton(operJoystick, 2).onTrue(new ArmHigh(armSubsystem));
    // new JoystickButton(operJoystick, 3).onTrue(new ArmZeroPosition(armSubsystem));
    new JoystickButton(xbox, 1).toggleOnTrue(new SwerveStop(swerveSubsystem));
  }

  public Command getAutonomousCommand(){
   
    PathPlannerTrajectory mTrajectory = PathPlanner.loadPath(
    "New New Path", 
    4,
    5);
    

      PIDController xController = new PIDController(
        3, 
        0, 
        0.1);
      PIDController yController = new PIDController(
        3, 
        0, 
        0.1);
      ProfiledPIDController thetaController = new ProfiledPIDController(
        0, 
        0, 
        0, 
        AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      //Swerve Auto Commands
      SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        mTrajectory,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem); 

      //Auto Routine
      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(mTrajectory.getInitialPose())),
        swerveControllerCommand1,
        new InstantCommand(() -> swerveSubsystem.stopModules())
      );
  }

}
