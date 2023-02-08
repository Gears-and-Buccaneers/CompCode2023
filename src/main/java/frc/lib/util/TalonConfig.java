package frc.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class TalonConfig extends TalonFXConfiguration {
	boolean invert;
	NeutralMode neutralMode;

	public TalonConfig(double kP, double kI, double kD, double kF, boolean invert, NeutralMode neutralMode,
			SupplyCurrentLimitConfiguration lim) {
		this.slot0.kP = kP;
		this.slot0.kI = kI;
		this.slot0.kD = kD;
		this.slot0.kF = kF;

		this.supplyCurrLimit = lim;
		this.initializationStrategy = SensorInitializationStrategy.BootToZero;

		this.invert = invert;
		this.neutralMode = neutralMode;
	}

	public TalonConfig(double kP, double kI, double kD, double kF, boolean invert, NeutralMode neutralMode,
			SupplyCurrentLimitConfiguration lim, double openLoopRamp, double closedloopRamp) {
		this(kP, kI, kD, kF, invert, neutralMode, lim);

		this.openloopRamp = openLoopRamp;
		this.closedloopRamp = closedloopRamp;
	}

	public TalonFX create(int deviceNumber, double sensorPosition) {
		TalonFX motor = new TalonFX(deviceNumber);

		motor.configFactoryDefault();
		motor.configAllSettings(this);

		motor.setInverted(invert);
		motor.setNeutralMode(neutralMode);

		motor.setSelectedSensorPosition(sensorPosition);

		return motor;
	}
}
