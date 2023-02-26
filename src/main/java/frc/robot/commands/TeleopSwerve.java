package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.Controls;
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
				new Translation2d(
						Controls.driver.LS_X.get(),
						Controls.driver.LS_Y.get())
						.times(Constants.SwerveConst.maxSpeedBoost)
						.times(Controls.driver.LT_S.get() >= .5 ? Controls.driver.LT_S.get() : .5),
				Controls.driver.RS_X.get() * Constants.SwerveConst.maxAngularVelocity,
				Controls.driver.LB.getAsBoolean(),
				Constants.SwerveConst.openLoop);
	}
}
