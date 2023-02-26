package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.CTREModuleState;
import frc.lib.math.Conversions;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public class SwerveModule {
	public int moduleNumber;
	public TalonFX angleMotor;
	public TalonFX driveMotor;
	private CANCoder angleEncoder;
	// public double lastAngle;
	private double angleOffset;

	public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
		this.moduleNumber = moduleNumber;
		this.angleOffset = angleOffset;

		/* Angle Encoder Config */
		angleEncoder = new CANCoder(canCoderID);
		configAngleEncoder();

		/* Angle Motor Config */
		angleMotor = Constants.SwerveConst.angleConfig.create(angleMotorID);

		/* Drive Motor Config */
		driveMotor = Constants.SwerveConst.driveConfig.create(driveMotorID);

		// updateAngleMotor();

		// lastAngle = getIntegrated();
	}

	/*
	 * public void updateAngleMotor() {
	 * double absolutePosition = Conversions.degreesToFalcon(
	 * getAbsolutePosition(),
	 * Constants.Swerve.angleGearRatio);
	 * angleMotor.setSelectedSensorPosition(absolutePosition);
	 * angleMotor.set(ControlMode.Position, absolutePosition);
	 * }
	 */

	public void setToCurrent() {
		angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition());
	}

	public double getAbsolutePosition() {
		return getEncoder() - angleOffset;
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromDegrees(getAbsolutePosition()));

		if (isOpenLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConst.maxSpeed;
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveConst.wheelCircumference,
					Constants.SwerveConst.driveGearRatio);

			driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
					Constants.SwerveConst.driveFeedforward.calculate(desiredState.speedMetersPerSecond));
		}

		double angle = getIntegrated() + (desiredState.angle.getDegrees() - getAbsolutePosition());

		// angle = Math.abs(lastAngle - angle) <= 2 ? lastAngle : angle;

		// Prevent rotating module if new angle is within 2 degrees of the last angle.
		// Prevents Jittering.
		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle,
				Constants.SwerveConst.angleGearRatio));

		// lastAngle = angle;
	}

	private void configAngleEncoder() {
		angleEncoder.configFactoryDefault();

		CANCoderConfiguration config = new CANCoderConfiguration();

		config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		config.sensorDirection = Constants.SwerveConst.canCoderInvert;
		config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		angleEncoder.configAllSettings(config);
	}

	public double getEncoder() {
		return angleEncoder.getAbsolutePosition();
	}

	public double getIntegrated() {
		return Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.SwerveConst.angleGearRatio);
	}

	public double getVelocity() {
		return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
				Constants.SwerveConst.wheelCircumference, Constants.SwerveConst.driveGearRatio);
	}

	public SwerveModulePosition getPos() {
		double distanceMeters = Conversions.falconToRPM(driveMotor.getSelectedSensorPosition(),
				Constants.SwerveConst.driveGearRatio);

		Rotation2d angle = Rotation2d.fromDegrees(getAbsolutePosition());

		return new SwerveModulePosition(distanceMeters, angle);
	}
}
