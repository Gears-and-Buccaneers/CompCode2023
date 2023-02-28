package frc.robot.autos;

import frc.robot.Constants.AutoC;
import frc.robot.Constants.SwerveC;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class PathPlannerAuto {
	// This is just an example event map. It would be better to have a constant,
	// global event map in your code that will be used by all path following
	// commands.
	HashMap<String, Command> eventMap = new HashMap<>();
	// eventMap.put("marker1", new PrintCommand("Passed marker 1"));
	// eventMap.put("intakeDown", new IntakeDown());

	SwerveAutoBuilder autoBuilder;

	public PathPlannerAuto(Swerve swerve) {
		autoBuilder = new SwerveAutoBuilder(
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
	}

	public void addAll(SendableChooser<Command> chooser) {
		File dir = new File(Filesystem.getDeployDirectory(), "pathplanner");
		for (File name : dir.listFiles()) {
			String item = name.toPath().getFileName().toString();
			chooser.addOption(item, get(item));
		}
	}

	public CommandBase get(String name) {

		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name,
				AutoC.constraints);

		return autoBuilder.fullAuto(pathGroup);
	}
}
