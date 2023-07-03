// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.concurrent.TimeUnit;

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
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClawSubsystem clawSubsystem = new ClawSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

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

    intakeSubsystem.setDefaultCommand(new IntakeZero(intakeSubsystem));
    configureButtonBindings();
  }

  public void configureButtonBindings(){

    new JoystickButton(xbox, 1).onTrue(new ElevatorZeroPosition(elevatorSubsystem));
    new JoystickButton(xbox, 2).onTrue(new ElevatorLowPosition(elevatorSubsystem));
    new JoystickButton(xbox, 3).onTrue(new ElevatorMidPosition(elevatorSubsystem));
    new JoystickButton(xbox, 4).onTrue(new ElevatorHighPosition(elevatorSubsystem));

    new JoystickButton(xbox, 5).whileTrue(new IntakeForward(intakeSubsystem));
    new JoystickButton(xbox, 6).whileTrue(new IntakeReverse(intakeSubsystem));

    new JoystickButton(xbox, 7).onTrue(new ClawClosed(clawSubsystem));
    new JoystickButton(xbox, 8).onTrue(new ClawOpen(clawSubsystem));

    new JoystickButton(operJoystick, 1).onTrue(new ArmMid(armSubsystem));
    new JoystickButton(operJoystick, 2).onTrue(new ArmHigh(armSubsystem));
    new JoystickButton(operJoystick, 3).onTrue(new ArmZeroPosition(armSubsystem));
  }

  public Command getAutonomousCommand(){
    
    Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(0, -1),
        new Translation2d(0, -1.5)
      ),
      new Pose2d(0, -2, new Rotation2d(Math.PI)), 
      new TrajectoryConfig(2, 1).setKinematics(DriveConstants.kDriveKinematics));

      Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, -2, new Rotation2d(Math.PI)), 
      List.of(
        new Translation2d(-1, -2),
        new Translation2d(-2, -2)
      ),
      new Pose2d(-3,-2, new Rotation2d(Math.PI/2)), 
      new TrajectoryConfig(2, 1).setKinematics(DriveConstants.kDriveKinematics));

      PIDController xController = new PIDController(1.3, 0, 0.05);
      PIDController yController = new PIDController(1.3, 0, 0.05);
      ProfiledPIDController thetaController = new ProfiledPIDController(1.3, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(
        trajectory1,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem); 

        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        trajectory2,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem); 

      return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory1.getInitialPose())),
        swerveControllerCommand1,
        new InstantCommand(() -> swerveSubsystem.stopModules()),
        swerveControllerCommand2,
        new InstantCommand(() -> swerveSubsystem.stopModules())
      );

      /* //Trajectory 2 (returning with cube)
      Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(-4.73, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(-3.4, 0.01),
        new Translation2d(-1.7, 0.01)
      ),
      new Pose2d(0, 0, new Rotation2d(Math.PI)), 
      new TrajectoryConfig(1.2, 1).setKinematics(DriveConstants.kDriveKinematics));

      //Trajectory 3 (going for second cube)
      Trajectory trajectory3 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(Math.PI)), 
      List.of(
        new Translation2d(-1.7, 0.01),
        new Translation2d(-2.9, 0.01),
        new Translation2d(-3.9, 0.6)
      ),
      new Pose2d(-4.73, 1.2192, new Rotation2d(Math.PI/6)), 
      new TrajectoryConfig(0.5, 1).setKinematics(DriveConstants.kDriveKinematics));

      //Trajectory 4 (returning with second cube)
      Trajectory trajectory4 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(-4.73, 1.2192, new Rotation2d(Math.PI/6)), 
      List.of(
        new Translation2d(-3.9, 0.6),
        new Translation2d(-2.9, 0.01),
        new Translation2d(-1.7, 0.01)
      ),
      new Pose2d(0, 0, new Rotation2d(Math.PI)), 
      new TrajectoryConfig(0.5, 1).setKinematics(DriveConstants.kDriveKinematics));

      //Trajectory 5 (moving to second scoring position)
      Trajectory trajectory5 = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(Math.PI)), 
      List.of(
        new Translation2d(0, 0.8)
      ),
      new Pose2d(0, 1.9, new Rotation2d(Math.PI)), 
      new TrajectoryConfig(0.5, 1).setKinematics(DriveConstants.kDriveKinematics));
      */

      /*SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        trajectory2,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem);

        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        trajectory3,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem);

        SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
        trajectory4,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem);

        SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
        trajectory5,
        swerveSubsystem::getPose, 
        DriveConstants.kDriveKinematics, 
        xController,
        yController,
        thetaController, 
        swerveSubsystem::setModuleStates, 
        swerveSubsystem); */
  }

}
