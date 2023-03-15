package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kSwerve;
import frc.robot.Constants.kBoom.Level;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;

public class SimpleAuto {
	public static CommandBase dropPiece(Boom arm, Gripper gripper, Level level) {
		return arm.setTo(Level.INTAKE).andThen(arm.setTo(level), gripper.runOnce(gripper::toggle),
				new WaitCommand(1), gripper.runOnce(gripper::toggle), new WaitCommand(1), arm.setTo(Level.BOTTOM));
	}

	public static CommandBase autoBalance(Swerve swerve) {
		return swerve.run(() -> swerve.drive(new Translation2d(-0.25, 0), 0, true, kSwerve.openLoop))
				.until(() -> Math.abs(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180)) > 2)
				.andThen(new AutoBalance(swerve), swerve.runOnce(() -> {
					swerve.drive(new Translation2d(), 0.01, true, false);
					swerve.drive(new Translation2d(), 0, true, false);
				}));
	}

	public static CommandBase go(Swerve swerve) {
		return swerve.run(() -> swerve.drive(new Translation2d(-0.5, 0), 0, true, kSwerve.openLoop))
				.withTimeout(2.1);
	}

}
