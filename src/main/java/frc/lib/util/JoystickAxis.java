package frc.lib.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickAxis implements Supplier<Double> {
	private final int axis;
	private final Joystick joystick;

	private final double deadband;

	public JoystickAxis(Joystick joystick, int axis, double deadband) {
		this.axis = axis;
		this.joystick = joystick;

		this.deadband = deadband;
	}

	public Double get() {
		double val = joystick.getRawAxis(axis);
		if (Math.abs(val) < deadband) {
			return 0.0;
		}
		return val;
	}
}
