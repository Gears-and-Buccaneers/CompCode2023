package frc.robot.commands;

import frc.lib.configs.Constants;
import frc.lib.util.JoystickAxis;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TeleopSwerve extends CommandBase {

	private JoystickButton fieldRelative;
	private boolean openLoop;

	private Swerve swerve;

	private JoystickAxis xAxis;
	private JoystickAxis yAxis;
	private JoystickAxis rAxis;

	/**
	 * Driver control
	 */
	public TeleopSwerve(Swerve swerve, JoystickAxis xAxis, JoystickAxis yAxis, JoystickAxis rAxis,
			JoystickButton fieldRelative, boolean openLoop) {
		this.swerve = swerve;
		addRequirements(swerve);

		this.xAxis = xAxis;
		this.yAxis = yAxis;
		this.rAxis = rAxis;
		this.fieldRelative = fieldRelative;
		this.openLoop = openLoop;
	}

	@Override
	public void execute() {
		swerve.drive(
				new Translation2d(yAxis.get(), xAxis.get()).times(Constants.Swerve.maxSpeed),
				rAxis.get() * Constants.Swerve.maxAngularVelocity,
				fieldRelative.getAsBoolean(), openLoop);
	}
}
