package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;

public class ConfigurablePID extends PIDController {
	public ConfigurablePID(double kP, double kI, double kD) {
		super(kP, kI, kD);
	}
}
