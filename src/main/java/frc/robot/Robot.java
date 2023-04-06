package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Controls;
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
	private final Boom boom = new Boom();
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

		SmartDashboard.putData("zero gyro", swerve.run(swerve::zeroGyro));
		SmartDashboard.putData("zero Boom encoder", boom.runOnce(boom::zeroEncoder));

		// Configure the button bindings
		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));

		Controls.operator.DOWN.onTrue(boom.setTo(Level.INTAKE));
		Controls.operator.LEFT.onTrue(boom.setTo(Level.BOTTOM));
		Controls.operator.UP.onTrue(boom.setTo(Level.MIDDLE));
		Controls.operator.RIGHT.onTrue(boom.setTo(Level.TOP));

		Controls.driver.RB.whileTrue(gripper.runOnce(gripper::toggle));

		compressor.enableDigital();

		makeAutos();
		System.out.println("hello world");
	}

	public void makeAutos() {
		// Setup autos picker
		chooser.setDefaultOption("None", null);

		// drop pice
		chooser.addOption("Drop piece (Top)",
				SimpleAuto.dropPiece(boom, gripper, Level.TOP));
		chooser.addOption("Drop piece (Mid)",
				SimpleAuto.dropPiece(boom, gripper, Level.MIDDLE));
		chooser.addOption("Drop piece (Bot)",
				SimpleAuto.dropPiece(boom, gripper, Level.BOTTOM));

		// drop pice then balance
		chooser.addOption("Drop piece (TOP) -> Auto-balance",
				SimpleAuto.dropPiece(boom, gripper, Level.TOP).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("Drop piece (Mid) -> Auto-balance",
				SimpleAuto.dropPiece(boom, gripper, Level.MIDDLE).andThen(SimpleAuto.autoBalance(swerve)));
		chooser.addOption("Drop piece (Bot) -> Auto-balance",
				SimpleAuto.dropPiece(boom, gripper, Level.BOTTOM).andThen(SimpleAuto.autoBalance(swerve)));

		// Drop Pice and the MOBILITY
		chooser.addOption("drop (Top)-> MOBILITY",
				SimpleAuto.dropPiece(boom, gripper, Level.TOP).andThen(SimpleAuto.exitCommunity(swerve)));
		chooser.addOption("drop (Mid)-> MOBILITY",
				SimpleAuto.dropPiece(boom, gripper, Level.MIDDLE).andThen(SimpleAuto.exitCommunity(swerve)));
		chooser.addOption("drop (Bot)-> MOBILITY",
				SimpleAuto.dropPiece(boom, gripper, Level.BOTTOM).andThen(SimpleAuto.exitCommunity(swerve)));

		// Mobilty
		chooser.addOption("MOBILITY",
				SimpleAuto.exitCommunity(swerve));
		chooser.addOption("Auto-balance",
				SimpleAuto.autoBalance(swerve));

		SmartDashboard.putData("Auto Path", chooser);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		SmartDashboard.putData(CommandScheduler.getInstance());
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
