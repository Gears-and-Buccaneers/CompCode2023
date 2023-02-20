package frc.robot;

// import javax.xml.crypto.KeySelector.Purpose;

// import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.configs.Constants.Controls;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.lib.configs.Constants.Pneumatics;

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

		// Controls.driver.Y.whileTrue(arm.run(() -> arm.set(5.0)));

		// Controls.driver.Y.whileTrue(new SetAngle(swerve, 0));
		// Controls.driver.B.whileTrue(new SetAngle(swerve, 90));
		// Controls.driver.A.whileTrue(new SetAngle(swerve, 180));
		// Controls.driver.X.whileTrue(new SetAngle(swerve, 270));

		// Configure the button bindings
		Controls.driver.BACK.whileTrue(swerve.runOnce(swerve::zeroGyro));// .andThen(swerve.runOnce(swerve::updateAngleMotors)));
		Controls.driver.X.whileTrue(new VisionTest(swerve));
		Controls.driver.Y.whileTrue(gripper.runOnce(gripper::toggle));

		// Setup autos picker
		chooser.setDefaultOption("None", null);
		chooser.addOption("Coded Trajectory", exampleAuto);
		chooser.addOption("PathPlannerAuto", PathPlannerAuto);

		SmartDashboard.putData("Auto Path", chooser);

		// set up compresser
		Pneumatics.compressor.enableHybrid(100, 120);
		SmartDashboard.putNumber("compressor PSO", Pneumatics.compressor.getPressure());
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
