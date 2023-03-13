package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
	private Swerve swerve;
	public PIDController pid = new PIDController(0.5, 0, .01);

	public AutoBalance(Swerve subsys) {
		swerve = subsys;
		pid.setTolerance(1, 1);
		pid.enableContinuousInput(-180, 180);
		SmartDashboard.putData("Autobalance PID", pid);
		addRequirements(swerve);
	}

	public void execute() {
		double speed = pid.calculate(MathUtil.inputModulus(swerve.getPitch().getDegrees(),
				-180, 180));

		swerve.drive(new Translation2d(speed * 0.05, 0), 0, false, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return pid.atSetpoint();
	}
}
