// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.configs;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.JoystickAxis;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class OIConstants {
	public static final class LogitechController {
		public static final int yAxis = 1;
		public static final int xAxis = 0;
		public static final int rotAxis = 4;

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

			LT_X = new JoystickAxis(joystick, 0);
			LT_Y = new JoystickAxis(joystick, 1);
			RT_X = new JoystickAxis(joystick, 2);
			RT_Y = new JoystickAxis(joystick, 3);
		}
	}

	public static final double deadband = 0.05;
	public static final LogitechController driver = new LogitechController(0);
	public static final LogitechController operator = new LogitechController(1);
}
