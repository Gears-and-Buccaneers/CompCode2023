package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.configs.Constants;
import frc.robot.subsystems.Swerve;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionTest extends CommandBase {
	private final Swerve swerve;
	private final PhotonCamera camera;

	final double ANGULAR_P = 0.1;
	final double ANGULAR_D = 0.0;

	PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

	public VisionTest(Swerve swerve) {
		this.swerve = swerve;
		this.camera = new PhotonCamera(Constants.Vision.cameraName);

	}

	@Override
	public void execute() {
		PhotonPipelineResult result = camera.getLatestResult();

		double speed = 0;
		// result.getBestTarget().getFiducialId()
		if (result.hasTargets())
			speed = -turnController.calculate(result.getBestTarget().getYaw(), 0);

		swerve.drive(new Translation2d(), speed * 0.75, false, Constants.Swerve.openLoop);
	}
}
