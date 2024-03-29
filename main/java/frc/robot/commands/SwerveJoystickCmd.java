package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.driveConstants;
import frc.robot.subsystems.SwerveSubystem;

public class SwerveJoystickCmd extends Command {
    //intializing supplire variables for joystick commands
    private final SwerveSubystem swerveSubystem;
    private Supplier<Double> xSpdFunction, ySpdFunction, angleSpdFunction;
    private Supplier<Boolean> fieldOrientedFunction;
    //intializing limiters
    private SlewRateLimiter xLimiter, yLimiter, angleLimiter;

    //constructor setting variables for the commands to themselves and adding a requirement 
    public SwerveJoystickCmd(SwerveSubystem swerveSubystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> isFieldOrientedFunction)
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
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double angleSpeed = angleSpdFunction.get();

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
        if (fieldOrientedFunction.get())
        {
            //relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angleSpeed, swerveSubystem.getRotation2d());
        } else {
            //relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angleSpeed);
        }
    }
    
}
