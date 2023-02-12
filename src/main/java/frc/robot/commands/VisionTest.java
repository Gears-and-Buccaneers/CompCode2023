package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.configs.Constants;
import frc.robot.subsystems.Swerve;

import org.photonvision.PhotonCamera;

public class VisionTest extends CommandBase {
	private final Swerve swerve;
	private final PhotonCamera camera;

	public VisionTest(Swerve swerve) {
		this.swerve = swerve;
		this.camera = new PhotonCamera(Constants.Vision.cameraName);
	}

	@Override
	public void execute() {
		var result = camera.getLatestResult();

		if (result.hasTargets()) {
			swerve.drive(new Translation2d(), result.getBestTarget().getYaw(), false, Constants.Swerve.openLoop);
		}
	}
}
