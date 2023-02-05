package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.configs.OIConstants;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
	private final Swerve swerve = new Swerve();

	/* Autos */
	SendableChooser<Command> chooser = new SendableChooser<>();

	private final Command simpleAuto = null;
	private final Command complexAuto = new exampleAuto(swerve);
	private final Command PathPlannerAuto = new PathPlanerTesting(swerve);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		swerve.setDefaultCommand(
				new TeleopSwerve(swerve, OIConstants.driver.LT_X, OIConstants.driver.LT_Y,
						OIConstants.driver.RT_X, OIConstants.driver.LB, true)); // conter clockwise +

		// Configure the button bindings
		configureButtonBindings();

		// setup Autos
		chooser.setDefaultOption("Simple Auto", simpleAuto);
		chooser.addOption("Coded Trajectory", complexAuto);
		chooser.addOption("PathPlannerAuto", PathPlannerAuto);

		SmartDashboard.putData(chooser);
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		OIConstants.driver.BACK.whileTrue(new InstantCommand(swerve::zeroGyro));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return chooser.getSelected();
	}
}
