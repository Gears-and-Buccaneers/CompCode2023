package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;

public class Vision extends SubsystemBase {
	SwerveDrivePoseEstimator estimator;

	@Override
	public void periodic() {
		for (Camera cam : kVision.cameras) {
			PhotonPipelineResult result = cam.c.getLatestResult();

			Pose2d pose = reduce(result.getTargets());

			estimator.addVisionMeasurement(pose, 0);
		}

		Map<Integer, Transform3d> res = new HashMap<>();

		for (Entry<Integer, Reducer> entry : data.entrySet())
			res.put(entry.getKey(), entry.getValue().average());

		return res;
	}

	static Pose2d reduce(List<PhotonTrackedTarget> targets) {
		double x = 0, y = 0, r = 0;

		for (PhotonTrackedTarget target : targets)
			if (target.getPoseAmbiguity() != -1 && target.getPoseAmbiguity() <= kVision.maxAmbiguity) {

				Reducer entry = data.get(target.getFiducialId());

				if (entry == null) {
					entry = new Reducer();
					data.put(target.getFiducialId(), entry);
				}

				entry.add(kVision.maxAmbiguity - target.getPoseAmbiguity(), target.getBestCameraToTarget().plus(cam.rel));
			}
	}

	public static class Reducer {
		double total = 0, x = 0, y = 0, r = 0;

		Transform3d average() {
			return new Transform3d(translation.div(total), new Rotation3d(x / total, y / total, z / total));
		}

		void add(double confidence, Transform3d other) {
			translation = translation.plus(other.getTranslation().times(confidence));

			Rotation3d r = other.getRotation();
			x += r.getX() * confidence;
			y += r.getY() * confidence;
			z += r.getZ() * confidence;

			total += confidence;
		}
	}

	public static class Camera {
		PhotonCamera c;
		Transform3d rel;

		public Camera(String name, Transform3d robotRelative) {
			c = new PhotonCamera(name);
			rel = robotRelative;
		}
	}
}
