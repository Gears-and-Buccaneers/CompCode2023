package frc.robot.autos;

import frc.robot.Constants.AutoC;
import frc.robot.Constants.SwerveC;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;

import java.io.File;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class PathPlannerAuto {
	static HashMap<String, Command> eventMap = new HashMap<>();

	public static void addAll(Swerve swerve, SendableChooser<Command> chooser) {
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
				swerve::getPose, // Pose2d supplier
				swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
				SwerveC.swerveKinematics, // SwerveDriveKinematics
				AutoC.translation, // PID constants to correct for translation error (used to create the X
				// and Y PID controllers)
				AutoC.rotation, // PID constants to correct for rotation error (used to create the
				// rotation controller)
				swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
				eventMap,
				swerve // The drive subsystem. Used to properly set the requirements of path following
		// commands
		);

		File dir = new File(Filesystem.getDeployDirectory(), "pathplanner");
		for (File item : dir.listFiles()) {
			String name = item.toPath().getFileName().toString().replace(".path", "");
			chooser.addOption(name, autoBuilder.fullAuto(PathPlanner.loadPathGroup(name, AutoC.constraints)));
		}
	}
}
