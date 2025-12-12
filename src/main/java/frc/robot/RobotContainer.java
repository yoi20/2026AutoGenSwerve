// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Vision-align tuning
    private static final double kVisionKP = 1.0; // proportional gain for rotation (rad/s per rad error)
    private static final double kVisionDeadbandDeg = 1.0; // degrees of allowable error
    private static final double kApproachTaThreshold = 2.0; // target area threshold to start/stop approaching (percent)
    private static final double kApproachSpeed = 0.25; // m/s forward approach speed

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // limelight test to align on right bumper
        joystick.rightBumper().onTrue(Commands.runOnce(() -> System.out.println("Right bumper pressed (onTrue)")));
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> {
                double tv = LimelightHelpers.getLimelightNTDouble("limelight-a", "tv");
                double tx = LimelightHelpers.getTX("limelight-a");
                double ta = LimelightHelpers.getTA("limelight-a");
                System.out.printf("[Limelight] RB held -> tv=%.2f tx=%.2f ta=%.2f\n", tv, tx, ta);
                SmartDashboard.putNumber("Limelight/tv", tv);
                SmartDashboard.putNumber("Limelight/tx", tx);
                SmartDashboard.putNumber("Limelight/ta", ta);

                boolean hasTarget = tv >= 1.0;
                double rotRate = 0.0;
                double vx = 0.0;
                double vy = 0.0;

                if (hasTarget) {
                    double txDeg = LimelightHelpers.getTX("limelight-a");
                    if (Math.abs(txDeg) > kVisionDeadbandDeg) {                        
                        double errRad = Math.toRadians(txDeg);
                        rotRate = -kVisionKP * errRad;
                    }

                    
                    double ta = LimelightHelpers.getTA("limelight-a");
                    if (ta > 0 && ta < kApproachTaThreshold) {
                        vx = -kApproachSpeed; 
                    }
                }

                return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(rotRate);
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("testAuto");
    }
}
