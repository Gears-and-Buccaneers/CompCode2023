package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class PieceAlign extends CommandBase {
	private final Swerve swerve;

	private final PIDController xController = new PIDController(3, 0, 3);
	private final PIDController yController = new PIDController(3, 0, 3);
	private final PIDController rController = new PIDController(2, 0, 0);

	public PieceAlign(Swerve swerve) {
		this.swerve = swerve;

		xController.setSetpoint(target.getX());
		yController.setSetpoint(target.getY());
		rController.setSetpoint(target.getRotation().getZ());

		xController.setTolerance(0.2);
		yController.setTolerance(0.2);
		rController.setTolerance(Units.degreesToRadians(3));

		rController.enableContinuousInput(-Math.PI, Math.PI);

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		Pose2d current = swerve.getPose();

		swerve.drive(new Translation2d(
				xController.calculate(current.getX()),
				yController.calculate(current.getY())),
				rController.calculate(current.getRotation().getRadians()), true, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
	}
}
