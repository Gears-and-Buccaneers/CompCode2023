package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import java.io.IOException;

import com.nosolojava.fsm.runtime.Context;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineEngine;
import com.nosolojava.fsm.impl.runtime.basic.BasicStateMachineFramework;
import com.nosolojava.fsm.runtime.StateMachineEngine;
import com.nosolojava.fsm.runtime.executable.CustomAction;
import com.nosolojava.fsm.model.config.exception.ConfigurationException;
import com.nosolojava.fsm.parser.exception.SCXMLParserException;
import com.nosolojava.fsm.parser.XppActionParser;

public class StateMachine extends SubsystemBase implements XppActionParser {
	Context ctx;

	public StateMachine() {
		try {
			List<XppActionParser> actionParsers = new ArrayList<XppActionParser>();
			actionParsers.add(this);
			BasicStateMachineFramework.DEBUG.set(true);
			StateMachineEngine engine = new BasicStateMachineEngine(actionParsers);
			engine.start();
			ctx = engine.startFSMSession(Constants.State.file.toURI());
		} catch (ConfigurationException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (SCXMLParserException e) {
			e.printStackTrace();
		}
	}

	@Override
	public String getNamespace() {
		return Constants.State.namespace;
	}

	@Override
	public CustomAction parseAction(XmlPullParser xpp) throws XmlPullParserException, IOException {
		CustomAction result = null;
		switch (xpp.getName()) {
			case "enableDriving":
				Boolean drivingEnabled = xpp.getAttributeValue(null, "value").equals("1");
				result = new CustomAction() {
					@Override
					public void run(Context context) {
						// robot.drivingEnabled = drivingEnabled;
						if (drivingEnabled) {
							System.out.println("...enabled driving");
						} else {
							System.out.println("...disabled driving");
						}
					}
				};
				break;

			case "robotToggle":
				String piece = xpp.getAttributeValue(null, "key");
				Boolean open = xpp.getAttributeValue(null, "value").equals("1");
				result = new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: tell the robot to control the arm/gripper/boom
						System.out.println("Should " + (open ? "open/raise" : "close/lower") + " " + piece);
					}
				};
				break;

			case "doMagic":
				String magicKind = xpp.getAttributeValue(null, "magic");
				result = new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: make the robot do the right kind of magic
						System.out.println("Time to start doing magic: " + magicKind);
					}
				};
				break;

			case "calibrate":
				result = new CustomAction() {
					@Override
					public void run(Context context) {
						// TODO: tell the robot to calibrate
						System.out.println("Time to calibrate!");
					}
				};
				break;

			// This is for silly trace debugging, and probably encourages bad habits
			case "output":
				String message = xpp.getAttributeValue(null, "message");
				result = new CustomAction() {
					@Override
					public void run(Context context) {
						System.out.println("\n" + message + "\n===================");
					}
				};
				break;

			default:
				break;
		}
		return result;
	}

}
