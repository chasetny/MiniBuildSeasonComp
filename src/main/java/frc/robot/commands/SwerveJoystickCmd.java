package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem, 
        Supplier<Double> xSpdFunction, 
        Supplier<Double> ySpdFunction, 
        Supplier<Double> turningSpdFunction){

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xLimiter = new SlewRateLimiter(3);
        this.yLimiter = new SlewRateLimiter(3);
        this.turningLimiter = new SlewRateLimiter(3);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

        double xSpeed = xLimiter.calculate(MathUtil.applyDeadband(xSpdFunction.get(), 0.07)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double ySpeed = yLimiter.calculate(MathUtil.applyDeadband(ySpdFunction.get(), 0.02)) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        double turningSpeed = turningLimiter.calculate(MathUtil.applyDeadband(turningSpdFunction.get(), 0.02)) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
      

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
