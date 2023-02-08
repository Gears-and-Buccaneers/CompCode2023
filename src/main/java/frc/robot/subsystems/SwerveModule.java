package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.configs.Constants;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;

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
	private double lastAngle;

	public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID, double angleOffset) {
		this.moduleNumber = moduleNumber;

		/* Angle Encoder Config */
		angleEncoder = new CANCoder(canCoderID);
		configAngleEncoder();

		/* Angle Motor Config */
		double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset,
				Constants.Swerve.angleGearRatio);
		angleMotor = Constants.Swerve.angleConfig.create(angleMotorID, absolutePosition);

		/* Drive Motor Config */
		driveMotor = Constants.Swerve.driveConfig.create(driveMotorID, 0);

		lastAngle = getState().angle.getDegrees();
	}

	public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
		desiredState = CTREModuleState.optimize(desiredState, getState().angle);

		if (Constants.Swerve.openLoop) {
			double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
			driveMotor.set(ControlMode.PercentOutput, percentOutput);
		} else {
			double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference,
					Constants.Swerve.driveGearRatio);

			driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
					Constants.Swerve.driveFeedforward.calculate(desiredState.speedMetersPerSecond));
		}

		double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle
				: desiredState.angle.getDegrees(); // Prevent rotating module if speed is less then 1%. Prevents Jittering.
		angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio));
		lastAngle = angle;
	}

	private void configAngleEncoder() {
		angleEncoder.configFactoryDefault();

		CANCoderConfiguration config = new CANCoderConfiguration();

		config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
		config.sensorDirection = Constants.Swerve.canCoderInvert;
		config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
		config.sensorTimeBase = SensorTimeBase.PerSecond;

		angleEncoder.configAllSettings(config);
	}

	public Rotation2d getCanCoder() {
		return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
	}

	public SwerveModuleState getState() {
		double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(),
				Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);

		Rotation2d angle = Rotation2d.fromDegrees(
				Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));

		return new SwerveModuleState(velocity, angle);
	}

	public SwerveModulePosition getPos() {
		// double velocity =
		// Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(),
		// Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
		double distanceMeters = Conversions.falconToRPM(driveMotor.getSelectedSensorPosition(),
				Constants.Swerve.driveGearRatio);

		Rotation2d angle = Rotation2d.fromDegrees(
				Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));

		return new SwerveModulePosition(distanceMeters, angle);
	}

}
