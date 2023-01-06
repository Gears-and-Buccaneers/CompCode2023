package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Constants
import frc.lib.configs.OIConstants;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	/** Controlers */
	// private final LogitechController driver = new
	// LogitechController(Constants.driverPort);
	// private final LogitechController operator = new
	// LogitechController(Constants.operatorPort);

	private final Swerve swerve = new Swerve();

	/* Autos */
	SendableChooser<Command> chooser = new SendableChooser<>();

	private final Command simpleAuto = null;
	private final Command complexAuto = new exampleAuto(swerve);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		swerve.setDefaultCommand(
				new TeleopSwerve(swerve, OIConstants.driver.LT_X, OIConstants.driver.LT_Y,
						OIConstants.driver.RT_X, true, true));

		// Configure the button bindings
		configureButtonBindings();

		// setup Autos
		chooser.setDefaultOption("Simple Auto", simpleAuto);
		chooser.addOption("Complex Auto", complexAuto);
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
		OIConstants.driver.BACK.whenPressed(new InstantCommand(swerve::zeroGyro));
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
