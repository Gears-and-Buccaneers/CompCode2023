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
	// private final PhotonCamera leftCamera = new
	// PhotonCamera(kVision.leftCameraName);
	// private final PhotonCamera rightCamera = new
	// PhotonCamera(kVision.rightCameraName);

	// Subsystems
	private final Swerve swerve = new Swerve();
	private final Boom arm = new Boom();
	private final Gripper gripper = new Gripper();
	// private final PoseEstimator poseEstimator = new PoseEstimator(leftCamera,
	// swerve);

	// Commands

	// Autos
	private SendableChooser<Command> chooser = new SendableChooser<>();
	private Command autonomousCommand;

	// PathPlannerAuto pathPlanner = new PathPlannerAuto();

	private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

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

		compressor.enableDigital();

		makeAutos();
	}

	public void makeAutos() {
		// Setup autos picker
		chooser.setDefaultOption("None", null);

		// drop pice
		chooser.addOption("Drop piece (Top)",
				SimpleAuto.dropPiece(arm, gripper, Level.TOP));
		chooser.addOption("Drop piece (Mid)",
				SimpleAuto.dropPiece(arm, gripper, Level.MIDDLE));
		chooser.addOption("Drop piece (Bot)",
				SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM));

		// drop pice then balance
		chooser.addOption("Drop piece (TOP) -> Auto-balance",
				SimpleAuto.dropPiece(arm, gripper, Level.TOP).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("Drop piece (Mid) -> Auto-balance",
				SimpleAuto.dropPiece(arm, gripper, Level.MIDDLE).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("Drop piece (Bot) -> Auto-balance",
				SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM).andThen(SimpleAuto.autoBalance(swerve)));

		// Drop Pice and the MOBILITY
		chooser.addOption("drop (Top)-> MOBILITY",
				SimpleAuto.dropPiece(arm, gripper, Level.TOP).andThen(SimpleAuto.Mobilty(swerve)));
		chooser.addOption("drop (Mid)-> MOBILITY",
				SimpleAuto.dropPiece(arm, gripper, Level.MIDDLE).andThen(SimpleAuto.Mobilty(swerve)));
		chooser.addOption("drop (Bot)-> MOBILITY",
				SimpleAuto.dropPiece(arm, gripper, Level.BOTTOM).andThen(SimpleAuto.Mobilty(swerve)));

		// Mobilty
		chooser.addOption("MOBILITY",
				SimpleAuto.Mobilty(swerve));

		SmartDashboard.putData("Auto Path", chooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {
		SimpleAuto.lockIn(swerve).schedule(); // this should lock in the robot whenever it is disabled
		compressor.disable(); // turns off the compressor
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = chooser.getSelected();
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
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
		compressor.enableDigital();
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
