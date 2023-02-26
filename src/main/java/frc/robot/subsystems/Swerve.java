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
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
	public SwerveDriveOdometry swerveOdometry;
	public PigeonIMU gyro;

	public Swerve() {
		gyro = new PigeonIMU(Constants.SwerveConst.pigeonID);
		gyro.configFactoryDefault();
		zeroGyro();

		swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConst.swerveKinematics, getYaw(), getPos());
	}

	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

		if (fieldRelative)
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());

		setModuleStates(Constants.SwerveConst.swerveKinematics.toSwerveModuleStates(speeds), isOpenLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConst.maxSpeed);

		for (SwerveModule mod : Constants.SwerveConst.mods)
			mod.setDesiredState(desiredStates[mod.moduleNumber], openLoop);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		setModuleStates(desiredStates, true);
	}

	public void setToCurrent() {
		for (SwerveModule mod : Constants.SwerveConst.mods)
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
		for (SwerveModule mod : Constants.SwerveConst.mods)
			states[mod.moduleNumber] = mod.getPos();
		return states;
	}

	public void zeroGyro() {
		gyro.setYaw(0);
	}

	public Rotation2d getYaw() {
		return (Constants.SwerveConst.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
				: Rotation2d.fromDegrees(gyro.getYaw());
	}

	public Rotation2d getPitch() {
		return (Constants.SwerveConst.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getPitch())
				: Rotation2d.fromDegrees(gyro.getPitch());
	}

	public Rotation2d getRoll() {
		return (Constants.SwerveConst.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getRoll())
				: Rotation2d.fromDegrees(gyro.getRoll());
	}

	@Override
	public void periodic() {
		swerveOdometry.update(getYaw(), getPos());

		for (SwerveModule mod : Constants.SwerveConst.mods) {
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Cancoder", mod.getEncoder());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Integrated", mod.getIntegrated());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Velocity", mod.getVelocity());
			SmartDashboard.putNumber("Mod" + mod.moduleNumber + " Absolute Position", mod.getAbsolutePosition());
			// SmartDashboard.putNumber("Mod" + mod.moduleNumber + " last angle",
			// mod.lastAngle);
		}
	}
}
