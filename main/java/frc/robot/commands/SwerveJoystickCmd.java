package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.driveConstants;
import frc.robot.subsystems.SwerveSubystem;

public class SwerveJoystickCmd extends Command {
    //intializing supplire variables for joystick commands
    private final SwerveSubystem swerveSubystem;
    private DoubleSupplier xSpdFunction, ySpdFunction, angleSpdFunction;
    private BooleanSupplier fieldOrientedFunction;
    //intializing limiters
    private SlewRateLimiter xLimiter, yLimiter, angleLimiter;

    //constructor setting variables for the commands to themselves and adding a requirement 
    public SwerveJoystickCmd(SwerveSubystem swerveSubystem, DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction, BooleanSupplier isFieldOrientedFunction)
    {
        this.swerveSubystem = swerveSubystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.angleSpdFunction = angleSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(driveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(driveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.angleLimiter = new SlewRateLimiter(driveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubystem);
    }

    @Override
    public void execute()
    {
        //getting real time joystick input
        double xSpeed = xSpdFunction.getAsDouble();
        double ySpeed = ySpdFunction.getAsDouble();
        double angleSpeed = angleSpdFunction.getAsDouble();

        //applying deadzone sensitivity for motor protection
        xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadband ? ySpeed : 0.0;
        angleSpeed = Math.abs(angleSpeed) > OperatorConstants.kDeadband ? angleSpeed : 0.0;

        //limiting acceleration for smoother driving
        xSpeed = xLimiter.calculate(xSpeed) * driveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * driveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        angleSpeed = angleLimiter.calculate(angleSpeed) * driveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond;

        //constructing chassis speeds based on if we want field oriented or not
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.getAsBoolean())
        {
            //relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angleSpeed, swerveSubystem.getRotation2d());
        } else {
            //relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
        }

        // converting the chassis speeds we want into individual commands for every single module with module states array
        SwerveModuleState[] moduleStates = driveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);

        //sending each value we have gotten to each swerve module
        swerveSubystem.stopModules();
    }
    
    
    @Override
    public void end(boolean interrupted)
    {
        //stopping the modules at the end
        swerveSubystem.stopModules();
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }    
}
