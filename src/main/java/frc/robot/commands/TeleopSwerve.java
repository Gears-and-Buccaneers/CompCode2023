package frc.robot.commands;

import frc.lib.configs.JoystickAxis;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {

	private double rotation;

	private Translation2d translation;

	private boolean fieldRelative;
	private boolean openLoop;

	private Swerve swerve;

	private JoystickAxis xAxis;
	private JoystickAxis yAxis;
	private JoystickAxis rAxis;

	/**
	 * Driver control
	 */
	public TeleopSwerve(Swerve swerve, JoystickAxis xAxis,
			JoystickAxis yAxis, JoystickAxis rAxis,
			boolean fieldRelative, boolean openLoop) {
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
		translation = new Translation2d(yAxis.get(), xAxis.get()).times(Constants.Swerve.maxSpeed);
		rotation = rAxis.get() * Constants.Swerve.maxAngularVelocity;
		swerve.drive(translation, rotation, fieldRelative, openLoop);
	}
}
