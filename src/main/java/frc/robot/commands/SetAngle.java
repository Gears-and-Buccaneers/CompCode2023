package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SetAngle extends CommandBase {
	Rotation2d r;
	Swerve swerve;

	public SetAngle(Swerve subsys, double angle) {
		swerve = subsys;
		r = Rotation2d.fromDegrees(angle);
		addRequirements(swerve);
	}

	public void execute() {
		SwerveModuleState[] states = {
				new SwerveModuleState(0, r),
				new SwerveModuleState(0, r),
				new SwerveModuleState(0, r),
				new SwerveModuleState(0, r)
		};

		swerve.setModuleStates(states);
	}

	public boolean isFinished() {
		return true;
	}
}
