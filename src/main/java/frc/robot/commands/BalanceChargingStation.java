package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;


public class BalanceChargingStation extends CommandBase{
	private Swerve swerve;
    private Rotation2d r = Rotation2d.fromDegrees(0);
    public double speed;
    public PIDController pid = new PIDController(0.5, 0, .01);

	public BalanceChargingStation(Swerve subsys, double angle) {
		swerve = subsys;
		addRequirements(swerve);
        pid.setTolerance(.2);
	}

	public void execute() {
        speed = pid.calculate(swerve.getPitch().getDegrees(), 0);
		SwerveModuleState[] states = {
				new SwerveModuleState(speed, r),
				new SwerveModuleState(0, r),
				new SwerveModuleState(0, r),
				new SwerveModuleState(0, r)
		};

		swerve.setModuleStates(states);
	}
}
