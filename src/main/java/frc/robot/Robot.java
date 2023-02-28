package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.Controls;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.Vision;
import frc.robot.Constants.BoomC.Level;

import frc.robot.autos.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Robot extends TimedRobot {
	// Cameras
	private final PhotonCamera leftCamera = new PhotonCamera(Vision.leftCameraName);
	private final PhotonCamera rightCamera = new PhotonCamera(Vision.rightCameraName);

	// subystems
	private final Swerve swerve = new Swerve();
	private final Boom arm = new Boom();
	private final Gripper gripper = new Gripper();
	private final PoseEstimator poseEstimator = new PoseEstimator(leftCamera, swerve);

	// Comands
	private final VisionTest gotoTag = new VisionTest(swerve, rightCamera, leftCamera, poseEstimator, 4);
	// autos
	// PathPlannerAuto autos = new PathPlannerAuto(swerve);

	SendableChooser<Command> chooser = new SendableChooser<>();
	private final Command exampleAuto = new ExampleAuto(swerve);

	private Command autonomousCommand;

	@Override
	public void robotInit() {
		swerve.setDefaultCommand(new TeleopSwerve(swerve));

		// Configure the button bindings
		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));

		Controls.driver.UP.whileTrue(arm.setTo(Level.INTAKE));
		Controls.driver.RIGHT.whileTrue(arm.setTo(Level.BOTTOM));
		Controls.driver.DOWN.whileTrue(arm.setTo(Level.MIDDLE));
		Controls.driver.LEFT.whileTrue(arm.setTo(Level.TOP));

		Controls.driver.X.whileTrue(gotoTag);
		Controls.driver.Y.whileTrue(gripper.runOnce(gripper::toggle));

		Controls.driver.A.whileTrue(gripper.runOnce(() -> gripper.set(Value.kOff)));
		Controls.driver.B.whileTrue(gripper.runOnce(() -> gripper.set(Value.kReverse)));

		// Setup autos picker
		chooser.setDefaultOption("None", null);
		chooser.addOption("Coded Trajectory", exampleAuto);

		// autos.addAll(chooser);

		SmartDashboard.putData("Auto Path", chooser);

		// set up compresser
		Pneumatics.compressor.enableHybrid(110, 120);
		SmartDashboard.putNumber("compressor PSI", Pneumatics.compressor.getPressure());
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
		if (autonomousCommand != null)
			autonomousCommand.schedule();
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
