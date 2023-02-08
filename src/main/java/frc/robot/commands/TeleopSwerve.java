package frc.robot.commands;

import frc.lib.configs.Constants;
import frc.lib.configs.Constants.Controls;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
	private Swerve swerve;

	public TeleopSwerve(Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	@Override
	public void execute() {
		swerve.drive(
				new Translation2d(Controls.xAxis.get(), Controls.yAxis.get()).times(Constants.Swerve.maxSpeed),
				Controls.rAxis.get() * Constants.Swerve.maxAngularVelocity,
				Controls.fieldRelative.getAsBoolean(), Constants.Swerve.openLoop);
	}
}
