package frc.robot.subsystems;

import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kVision;

public class Vision extends SubsystemBase {
	private final SwerveDrivePoseEstimator estimator;

	public Vision(SwerveDrivePoseEstimator estimator) {
		this.estimator = estimator;
	}

	@Override
	public void periodic() {
		for (Camera c : kVision.cameras) {
			Pair<Measurement, Double> m = c.calculatePose();
			Pose3d pose = m.getFirst().toPose();
			estimator.addVisionMeasurement(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getZ())),
					m.getSecond());
		}
	}

	public static Pose3d getPose() {
		return Arrays.stream(kVision.cameras)
				.map(Camera::calculatePose).map(Pair::getFirst)
				.reduce(Measurement::sum).get().toPose();
	}

	public static class Camera {
		private PhotonCamera camera;
		private Transform3d pos;

		public Camera(String name, Transform3d robotRelative) {
			camera = new PhotonCamera(name);
			pos = robotRelative;
		}

		/**
		 * @return estimated pose and timestamp
		 */
		public Pair<Measurement, Double> calculatePose() {
			PhotonPipelineResult result = camera.getLatestResult();

			return new Pair<Measurement, Double>(result.getTargets().stream()
					.filter((t) -> t.getPoseAmbiguity() != -1 && t.getPoseAmbiguity() <= kVision.maxAmbiguity)
					.map(Measurement::from).reduce(Measurement::sum).map((m) -> Measurement.transform(m, pos))
					.orElse(new Measurement()), result.getTimestampSeconds());
		}
	}

	public static class Measurement {
		private final Triple pos, rot;
		private final double a;

		private static class Triple {
			final double x, y, z;

			public Triple() {
				this(0, 0, 0);
			}

			public Triple(Translation3d t) {
				this(t.getX(), t.getY(), t.getZ());
			}

			public Triple(Rotation3d t) {
				this(t.getX(), t.getY(), t.getZ());
			}

			public Triple(double x, double y, double z) {
				this.x = x;
				this.y = y;
				this.z = z;
			}

			public static Triple mul(Triple lhs, double s) {
				return new Triple(lhs.x * s, lhs.y * s, lhs.z * s);
			}

			public Translation3d asTranslation() {
				return new Translation3d(x, y, z);
			}

			public Rotation3d asRotation() {
				return new Rotation3d(x, y, z);
			}

			public static Triple add(Triple lhs, Triple rhs) {
				return new Triple(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
			}
		}

		public Measurement() {
			this(new Triple(), new Triple(), 0);
		}

		private Measurement(Triple pos, Triple rot, double a) {
			this.pos = pos;
			this.rot = rot;
			this.a = a;
		}

		public static Measurement from(PhotonTrackedTarget target) {
			Pose3d targetPose = kVision.tags.get(target.getFiducialId());

			if (targetPose == null)
				return new Measurement();

			Transform3d cameraToTarget = target.getBestCameraToTarget();

			Pose3d cameraPose = targetPose.transformBy(cameraToTarget.inverse());

			double a = kVision.maxAmbiguity - target.getPoseAmbiguity();

			return new Measurement(Triple.mul(new Triple(cameraPose.getTranslation()), a),
					Triple.mul(new Triple(cameraPose.getRotation()), a), a);
		}

		public static Measurement sum(Measurement lhs, Measurement rhs) {
			return new Measurement(Triple.add(lhs.pos, rhs.pos), Triple.add(lhs.rot, rhs.rot), lhs.a + rhs.a);
		}

		public static Measurement transform(Measurement m, Transform3d transform) {
			return new Measurement(
					Triple.add(m.pos, Triple.mul(new Triple(transform.getTranslation()), m.a)),
					Triple.add(m.rot, Triple.mul(new Triple(transform.getRotation()), m.a)), m.a);
		}

		public Pose3d toPose() {
			return new Pose3d(Triple.mul(pos, 1 / a).asTranslation(), Triple.mul(rot, 1 / a).asRotation());
		}
	}
}
