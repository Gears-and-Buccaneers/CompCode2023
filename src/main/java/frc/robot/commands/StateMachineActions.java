package frc.robot.commands;

import java.io.IOException;

import com.nosolojava.fsm.parser.XppActionParser;
import com.nosolojava.fsm.runtime.executable.CustomAction;
import com.nosolojava.fsm.runtime.Context;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;

import frc.robot.Robot;

public class StateMachineActions implements XppActionParser {
    private static final String NS = "http://dawsonschool.org/robotics/2972";
    private Robot robot;

    public StateMachineActions(Robot ownerRobot) {
        robot = ownerRobot;
    }
    
    @Override
	public String getNamespace() {
		return NS;
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
                        robot.drivingEnabled = drivingEnabled;
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
                        System.out.println("Should "+(open ? "open/raise" : "close/lower")+" "+piece);
                    }
                };
            break;

            case "doMagic":
                String magicKind = xpp.getAttributeValue(null, "magic");
                result = new CustomAction() {
                    @Override
                    public void run(Context context) {
                        // TODO: make the robot do the right kind of magic
                        System.out.println("Time to start doing magic: "+magicKind);
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
                        System.out.println("\n"+message+"\n===================");
                    }
                };
            break;

            default:
            break;
        }
		return result;
	}
}
