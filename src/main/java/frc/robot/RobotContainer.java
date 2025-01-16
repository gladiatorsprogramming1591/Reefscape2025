// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
	
	private final CommandXboxController joystick1 = new CommandXboxController(0);
	private final CommandXboxController joystick2 = new CommandXboxController(1);
	private final CommandXboxController joystick3 = new CommandXboxController(2);

	private final int frontLeftIndex = 0;
	private final int frontRightIndex = 1;
	private final int backLeftIndex = 2;
	private final int backRightIndex = 3;

	private final double deadband = 0.10;

	
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	
	private final SendableChooser<Command> autoChooser;
	
	public RobotContainer() {
		autoChooser = AutoBuilder.buildAutoChooser("Tests");
		SmartDashboard.putData("Auto Mode", autoChooser);
		
		configureBindings();
	}
	
	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() ->
						drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
								.withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
								.withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
				)
		);
		
		joystick1.a().whileTrue(drivetrain.applyRequest(() -> brake));
		joystick1.b().whileTrue(drivetrain.applyRequest(() ->
				point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))
		));
		
		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		joystick1.back().and(joystick1.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		joystick1.back().and(joystick1.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		joystick1.start().and(joystick1.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		joystick1.start().and(joystick1.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
		
		// reset the field-centric heading on left bumper press
		joystick1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.getModule(frontLeftIndex).getDriveMotor().set(joystick2.getLeftY() >= deadband || joystick2.getLeftY() <=  -deadband ? joystick2.getLeftY() : 0);
		drivetrain.getModule(frontLeftIndex).getSteerMotor().set(joystick2.getLeftX() >= deadband || joystick2.getLeftX() <=  -deadband ? joystick2.getLeftX() : 0);	
		drivetrain.getModule(backLeftIndex).getDriveMotor().set(joystick2.getRightY() >= deadband || joystick2.getRightY() <= -deadband ? joystick2.getRightY() : 0);	
		drivetrain.getModule(backLeftIndex).getSteerMotor().set(joystick2.getRightX() >= deadband || joystick2.getRightX() <= -deadband ? joystick2.getRightX() : 0);

		drivetrain.getModule(frontRightIndex).getDriveMotor().set(joystick3.getLeftY() >= deadband || joystick3.getLeftY() <=  -deadband ? joystick3.getLeftY() : 0);
		drivetrain.getModule(frontRightIndex).getSteerMotor().set(joystick3.getLeftX() >= deadband || joystick3.getLeftX() <=  -deadband ? joystick3.getLeftX() : 0);
		drivetrain.getModule(backRightIndex).getDriveMotor().set(joystick3.getRightY() >= deadband || joystick3.getRightY() <= -deadband ? joystick3.getRightY() : 0);
		drivetrain.getModule(backRightIndex).getSteerMotor().set(joystick3.getRightX() >= deadband || joystick3.getRightX() <= -deadband ? joystick3.getRightX() : 0);
		// drivetrain.getModule(backRightIndex).getSteerMotor().set(applyScaledDeadband(joystick3.getRightX(), deadband));

		drivetrain.registerTelemetry(logger::telemeterize);
	}

	// Example of overloaded method
	private double applyScaledDeadband(double value, double deadband)
	{
			return applyScaledDeadband(value, deadband, false);
	}

	private double applyScaledDeadband(double value, double deadband, boolean isSquared)
	{
		// Short-circuit if value is within deadband
		if (Math.abs(value) <= deadband)
		{
			return 0;
		}
		// Positive or negative 1 dependant on the sign of the value passed in
		double sign = value/Math.abs(value); 
		// Scales result to have full range between 0 and 1 on both sides of the deadband
		// Motor output = Function (e.g. y = x-0.10) * Scaler (the reciprocal of the function's max output to ensure it's always 1 (100%). e.g. if the max is 0.90, 0.90 * 1/0.90 = 1)
		double result = (value-(sign*deadband))*(1/(1-deadband));
		return isSquared ? Math.pow(result, 2) : result;
	}
	
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
