package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Controls;
import frc.robot.Constants.kVision;
import frc.robot.Constants.kBoom.Level;

import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Robot extends TimedRobot {
	// Cameras
	private final PhotonCamera leftCamera = new PhotonCamera(kVision.leftCameraName);
	private final PhotonCamera rightCamera = new PhotonCamera(kVision.rightCameraName);

	// Subsystems
	private final Swerve swerve = new Swerve();
	private final Boom arm = new Boom();
	private final Gripper gripper = new Gripper();
	// private final PoseEstimator poseEstimator = new PoseEstimator(leftCamera,
	// swerve);

	// Commands
	// private final VisionTest gotoTag = new VisionTest(swerve, rightCamera,
	// leftCamera, poseEstimator, 4);

	SendableChooser<Command> chooser = new SendableChooser<>();
	private Command autonomousCommand;
	PathPlannerAuto pathPlanner = new PathPlannerAuto();

	// private NetworkTableInstance inst = NetworkTableInstance.getDefault();
	// NetworkTable autoTab = inst.getTable("Auto");

	@Override
	public void robotInit() {
		swerve.setDefaultCommand(new TeleopSwerve(swerve));

		// Configure the button bindings
		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));

		Controls.operator.DOWN.onTrue(arm.setTo(Level.INTAKE));
		Controls.operator.LEFT.onTrue(arm.setTo(Level.BOTTOM));
		Controls.operator.UP.onTrue(arm.setTo(Level.MIDDLE));
		Controls.operator.RIGHT.onTrue(arm.setTo(Level.TOP));

		Controls.driver.RB.whileTrue(gripper.runOnce(gripper::toggle));

		// Setup autos picker
		chooser.setDefaultOption("None", null);
		// chooser.addOption("Coded Trajectory", new ExampleAuto(swerve));
		chooser.addOption("Straight", pathPlanner.get("GO Straight", swerve, Constants.kAuto.testingAuto));
		// chooser.addOption("Testing", pathPlanner.get("PathPlanerSwerve"));
		chooser.addOption("Drop piece (TOP)", SimpleAuto.dropPiece(arm, gripper, Level.TOP));
		chooser.addOption("Drop piece -> Auto-balance",
				SimpleAuto.dropPiece(arm, gripper, Level.TOP).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("Drop piece (LOWER)", SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM));
		chooser.addOption("Drop piece (LOWER) -> Auto-balance",
				SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("drop (LOWER)->Go",
				SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM).andThen(SimpleAuto.go(swerve)));
		chooser.addOption("drop->Go", SimpleAuto.dropPiece(arm, gripper, Level.TOP).andThen(SimpleAuto.go(swerve)));
		chooser.addOption("drop->Go111", SimpleAuto.go(swerve));
		// chooser.addOption("Autobalance (UNTESTED)",
		// SimpleAuto.autoBalanceUntested(swerve));

		SmartDashboard.putData("Auto Path", chooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
			// if (autonomousCommand)
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void simulationInit() {
	}

	@Override
	public void simulationPeriodic() {
	}
}
