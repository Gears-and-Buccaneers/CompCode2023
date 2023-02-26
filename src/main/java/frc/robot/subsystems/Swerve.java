package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveC;

public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry swerveOdometry;
	public PigeonIMU gyro;

	public Swerve() {
		gyro = new PigeonIMU(SwerveC.pigeonID);
		gyro.configFactoryDefault();
		zeroGyro();

		swerveOdometry = new SwerveDriveOdometry(SwerveC.swerveKinematics, getYaw(), getPos());
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		if (fieldRelative)
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());

		setModuleStates(SwerveC.swerveKinematics.toSwerveModuleStates(speeds), isOpenLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveC.maxSpeed);

		for (SwerveModule mod : SwerveC.mods)
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		setModuleStates(desiredStates, true);
	}

	public void setToCurrent() {
		for (SwerveModule mod : SwerveC.mods)
			mod.setToCurrent();
	}

	public Pose2d getPose() {
		return swerveOdometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		swerveOdometry.resetPosition(getYaw(), getPos(), pose);
	}

	public SwerveModulePosition[] getPos() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (SwerveModule mod : SwerveC.mods)
			states[mod.moduleNumber] = mod.getPos();
		return states;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	public Rotation2d getYaw() {
		return (SwerveC.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
				: Rotation2d.fromDegrees(gyro.getYaw());
	}

	public Rotation2d getPitch() {
		return (SwerveC.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getPitch())
				: Rotation2d.fromDegrees(gyro.getPitch());
	}

	public Rotation2d getRoll() {
		return (SwerveC.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll())
				: Rotation2d.fromDegrees(gyro.getRoll());
	}

	@Override
	public void periodic() {
		swerveOdometry.update(getYaw(), getPos());

		for (SwerveModule mod : SwerveC.mods) {
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Cancoder", mod.getEncoder());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Integrated", mod.getIntegrated());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Velocity", mod.getVelocity());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Absolute Position", mod.getAbsolutePosition());
			// SmartDashboard.putNumber("Mod" + mod.moduleNumber + " last angle",
			// mod.lastAngle);
		}
	}
}
