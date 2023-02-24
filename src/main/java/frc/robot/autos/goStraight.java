package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import com.pathplanner.lib.auto.PIDConstants;

public class goStraight extends SequentialCommandGroup {
	public goStraight(Swerve Swerve) {

		// This will load the file "FullAuto.path" and generate it with a max velocity
		// of 4 m/s and a max acceleration of 3 m/s^2
		// for every path in the group
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("GO Straight",
				new PathConstraints(.5, 3));

		// This is just an example event map. It would be better to have a constant,
		// global event map

		// in your code that will be used by all path following commands.
		HashMap<String, Command> eventMap = new HashMap<>();
		// eventMap.put("marker1", new PrintCommand("Passed marker 1"));
		// eventMap.put("intakeDown", new IntakeDown());

		// Create the AutoBuilder. This only needs to be created once when robot code
		// starts, not every time you want to create an auto command. A good place to
		// put this is in RobotContainer along with your subsystems.
		SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
				Swerve::getPose, // Pose2d supplier
				Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
				Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
				new PIDConstants(.7, 0.0, 0.2), // PID constants to correct for translation error (used to create the X
				// and Y PID controllers)
				new PIDConstants(1, 0.0, 0.1), // PID constants to correct for rotation error (used to create the
				// rotation controller)
				Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
				eventMap,
				Swerve // The drive subsystem. Used to properly set the requirements of path following
		// commands
		);

		Command fullAuto = autoBuilder.fullAuto(pathGroup);
		addCommands(fullAuto);
	}
}
