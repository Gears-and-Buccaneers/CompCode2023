package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.net.URI;
import java.io.IOException;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.configs.Constants.Controls;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineEngine;
import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineFramework;
import com.nosolojava.fsm.runtime.StateMachineEngine;
import com.nosolojava.fsm.runtime.Context;
import com.nosolojava.fsm.model.config.exception.ConfigurationException;
import com.nosolojava.fsm.parser.exception.SCXMLParserException;
import com.nosolojava.fsm.parser.XppActionParser;

public class Robot extends TimedRobot {
	private final Swerve swerve = new Swerve();
	public Context stateMachineContext;
	public Boolean drivingEnabled = false;

	SendableChooser<Command> chooser = new SendableChooser<>();

	private final Command exampleAuto = new exampleAuto(swerve);
	private final Command PathPlannerAuto = new PathPlannerTesting(swerve);

	private Command autonomousCommand;

	@Override
	public void robotInit() {
		// Boot up the state machine
		try {
			List<XppActionParser> actionParsers = new ArrayList<XppActionParser>();
			actionParsers.add(new StateMachineActions(this));
			BasicStateMachineFramework.DEBUG.set(true);
			StateMachineEngine engine = new BasicStateMachineEngine(actionParsers);
			engine.start();
			// FIXME: convert this to a classpath URI by adding src/main/assets/* */ or maybe just /assets/* to the packaging
			stateMachineContext = engine.startFSMSession(URI.create("file:///Users/phrogz/Code/scxmltest/scxmltest/src/main/java/net/phrogz/scxml/test/robot.scxml"));
        } catch (ConfigurationException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (SCXMLParserException e) {
            e.printStackTrace();
        }

		swerve.setDefaultCommand(new TeleopSwerve(swerve));

		// Configure the button bindings
		Controls.zeroGyro.whileTrue(new InstantCommand(swerve::zeroGyro));
		Controls.visionTest.whileTrue(new VisionTest(swerve));

		// Setup autos
		chooser.setDefaultOption("None", null);
		chooser.addOption("Coded Trajectory", exampleAuto);
		chooser.addOption("PathPlannerAuto", PathPlannerAuto);

		SmartDashboard.putData(chooser);
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
