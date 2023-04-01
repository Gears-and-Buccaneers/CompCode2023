package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.Swerve;

public class PieceAlign extends CommandBase {
	private final Swerve swerve;

	private final PIDController xController = new PIDController(3, 0, 3);
	private final PIDController yController = new PIDController(3, 0, 3);
	private final PIDController rController = new PIDController(2, 0, 0);

	private final DoubleSubscriber coneX = NetworkTableInstance.getDefault().getDoubleTopic("vision/coneX")
			.subscribe(0);
	private final DoubleSubscriber coneY = NetworkTableInstance.getDefault().getDoubleTopic("vision/coneY")
			.subscribe(0);
	private final DoubleSubscriber coneCenter = NetworkTableInstance.getDefault().getDoubleTopic("vision/coneCenter")
			.subscribe(0);

	public PieceAlign(Swerve swerve) {
		this.swerve = swerve;

		SmartDashboard.putData("pieceAlign rotation-controller", rController);

		xController.setSetpoint(0.5);
		yController.setSetpoint(kVision.desiredConeY);
		rController.setSetpoint(0.5);

		xController.setTolerance(0.05);
		yController.setTolerance(0.05);
		rController.setTolerance(0.05);

		addRequirements(swerve);
	}

	@Override
	public void execute() {
		swerve.drive(new Translation2d(
		// xController.calculate(coneCenter.get()),
		// yController.calculate(coneY.get())
		), rController.calculate(coneX.get()), false, kSwerve.openLoop);
	}

	@Override
	public boolean isFinished() {
		return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
	}
}
