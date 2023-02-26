package frc.robot.autos;

import frc.robot.Constants.AutoC;
import frc.robot.Constants.SwerveC;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
	public exampleAuto(Swerve s_Swerve) {
		TrajectoryConfig config = new TrajectoryConfig(
				AutoC.kMaxSpeedMetersPerSecond,
				AutoC.kMaxAccelerationMetersPerSecondSquared)
				.setKinematics(SwerveC.swerveKinematics);

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				config);

		var thetaController = new ProfiledPIDController(
				AutoC.kPThetaController, 0, 0, AutoC.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
				exampleTrajectory,
				s_Swerve::getPose,
				SwerveC.swerveKinematics,
				new PIDController(AutoC.kPXController, 0, 0),
				new PIDController(AutoC.kPYController, 0, 0),
				thetaController,
				s_Swerve::setModuleStates,
				s_Swerve);

		addCommands(
				new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
				swerveControllerCommand);
	}
}
