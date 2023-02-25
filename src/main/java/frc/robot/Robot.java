package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Controls;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.Boom.Level;
import frc.robot.autos.*;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
	private final Swerve swerve = new Swerve();
	private final Boom arm = new Boom();
	private final Gripper gripper = new Gripper();

	SendableChooser<Command> chooser = new SendableChooser<>();

	private final Command exampleAuto = new exampleAuto(swerve);
	private final Command PathPlannerAuto = new PathPlannerTesting(swerve);

	private Command autonomousCommand;

	@Override
	public void robotInit() {
		swerve.setDefaultCommand(new TeleopSwerve(swerve));

		Controls.driver.A.whileTrue(gripper.runOnce(gripper::toggle));
		Controls.driver.X.whileTrue(arm.setTo(Level.BOTTOM));
		Controls.driver.Y.whileTrue(arm.setTo(Level.TOP));

		// Configure the button bindings
		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));// .andThen(swerve.runOnce(swerve::updateAngleMotors)));
		// Controls.driver.X.whileTrue(new VisionTest(swerve));

		// Setup autos picker
		chooser.setDefaultOption("None", null);
		chooser.addOption("Coded Trajectory", exampleAuto);
		chooser.addOption("PathPlannerAuto", PathPlannerAuto);

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
