package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
	private Swerve swerve;
	public double speed;
	public PIDController pid = new PIDController(0.5, 0, .01);

	boolean finished;

	public AutoBalance(Swerve subsys) {

		swerve = subsys;
		pid.setTolerance(1);
		pid.setSetpoint(0);
		pid.enableContinuousInput(-180, 180);
		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		finished = false;
	}

	public void execute() {
		speed = pid.calculate(MathUtil.inputModulus(swerve.getPitch().getDegrees(),
				-180, 180));

		if (pid.atSetpoint()) {
			swerve.drive(new Translation2d(), 0.01, false, kSwerve.openLoop);
			new WaitCommand(0.1)
					.andThen(swerve.runOnce(() -> {
						swerve.drive(new Translation2d(), 0, false, kSwerve.openLoop);
						finished = true;
					}))
					.schedule();
		} else {
			swerve.drive(new Translation2d(speed * 0.05, 0), 0, false, kSwerve.openLoop);
		}
	}

	@Override
	public boolean isFinished() {
		return finished;
	}
}
