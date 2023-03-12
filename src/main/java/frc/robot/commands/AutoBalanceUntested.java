package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class AutoBalanceUntested extends CommandBase {
	private Swerve swerve;
	public double speed;
	public PIDController pid = new PIDController(0.5, 0, .01);

	int atLimitCount = 0;

	public AutoBalanceUntested(Swerve subsys) {
		swerve = subsys;
		pid.setTolerance(1);
		pid.setSetpoint(0);
		pid.enableContinuousInput(-180, 180);
		addRequirements(swerve);
	}

	public void execute() {
		speed = pid.calculate(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180));

		if (pid.atSetpoint())
			atLimitCount++;
		else
			atLimitCount = 0;

		swerve.drive(new Translation2d(speed * -0.05, 0), 0, true, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return atLimitCount > 25;
	}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) {
			swerve.drive(new Translation2d(), 0.01, false, kSwerve.openLoop);
			swerve.drive(new Translation2d(), 0, false, kSwerve.openLoop);
		}
	}
}