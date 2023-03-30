package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.subsystems.Swerve;

public class PieceAlign extends CommandBase {
	private final Swerve swerve;

	private final PIDController xController = new PIDController(3, 0, 3);
	private final PIDController yController = new PIDController(3, 0, 3);
	private final PIDController rController = new PIDController(2, 0, 0);

	private final DoubleSubscriber conePos = NetworkTableInstance.getDefault().getDoubleTopic("vision/conePos")
			.subscribe(0);

	public PieceAlign(Swerve swerve) {
		this.swerve = swerve;

		SmartDashboard.putData("pieceAlign rotation-controller", rController);

		// xController.setSetpoint(target.getX());
		// yController.setSetpoint(target.getY());
		rController.setSetpoint(0);

		// xController.setTolerance(0.2);
		// yController.setTolerance(0.2);
		rController.setTolerance(0.05);

		rController.enableContinuousInput(-1, 1);

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		// Pose2d current = swerve.getPose();

		swerve.drive(new Translation2d(/*
																		 * xController.calculate(current.getX()),
																		 * yController.calculate(current.getY())
																		 */),
				rController.calculate(conePos.get()), true, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
	}
}
