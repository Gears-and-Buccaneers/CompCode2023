package frc.lib.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickAxis implements Supplier<Double> {
	private final int axis;
	private final Joystick joystick;

	public JoystickAxis(Joystick joystick, int axis) {
		this.axis = axis;
		this.joystick = joystick;
	}

	public Double get() {
		return joystick.getRawAxis(axis);
	}
}
