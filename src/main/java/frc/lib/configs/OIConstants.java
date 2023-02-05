package frc.lib.configs;

import frc.lib.util.LogitechController;

public final class OIConstants {
	public static final double deadband = 0.05;

	public static final LogitechController driver = new LogitechController(0);
	public static final LogitechController operator = new LogitechController(1);
}
