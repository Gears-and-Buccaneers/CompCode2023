package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class LogitechController {
	public static final int yAxis = 1;
	public static final int xAxis = 0;
	public static final int rotAxis = 4;

	private static final double deadband = 0.05;

	public final JoystickButton A, B, X, Y, LB, RB, BACK, START, LEFT_THUMB, RIGHT_THUMB;
	public final JoystickAxis LT_X, LT_Y, RT_X, RT_Y;

	public LogitechController(int port) {
		Joystick joystick = new Joystick(port);

		A = new JoystickButton(joystick, 1);
		B = new JoystickButton(joystick, 2);
		X = new JoystickButton(joystick, 3);
		Y = new JoystickButton(joystick, 4);
		LB = new JoystickButton(joystick, 5);
		RB = new JoystickButton(joystick, 6);
		BACK = new JoystickButton(joystick, 7);
		START = new JoystickButton(joystick, 8);
		LEFT_THUMB = new JoystickButton(joystick, 9);
		RIGHT_THUMB = new JoystickButton(joystick, 10);

		LT_X = new JoystickAxis(joystick, 0, deadband);
		LT_Y = new JoystickAxis(joystick, 1, deadband);
		RT_X = new JoystickAxis(joystick, 4, deadband);
		RT_Y = new JoystickAxis(joystick, 5, deadband);
	}
}
