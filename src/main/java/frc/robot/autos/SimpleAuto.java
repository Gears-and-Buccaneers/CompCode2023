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
	public static CommandBase dropPiece(Boom boom, Gripper gripper, Level level) {
		return boom.setTo(Level.INTAKE).andThen(
				boom.setTo(level),
				gripper.runOnce(gripper::toggle),
				new WaitCommand(.5),
				gripper.runOnce(gripper::toggle),
				new WaitCommand(.5),
				boom.setTo(Level.BOTTOM));
	}

	public static CommandBase autoBalance(Swerve swerve) {
		return swerve.run(
				() -> swerve.drive(
						new Translation2d(-0.25, 0), 0, true, kSwerve.openLoop)) // drives backward at 25% speed
				.until(
						() -> Math.abs(MathUtil.inputModulus(swerve.getPitch().getDegrees(), -180, 180)) > 2)
				.withTimeout(2)
				.andThen(
						new AutoBalance(swerve), // then it runs the serve auto balace wich makes the robot leval
						lockIn(swerve) // then locks in the robot
				);
	}

	public static CommandBase lockIn(Swerve swerve) {
		return swerve.runOnce(() -> { // locks the robot in so it cant move.
			swerve.drive(new Translation2d(), 0.01, true, true);
			swerve.drive(new Translation2d(), 0, true, true);
		});
	}

	public static CommandBase exitCommunity(Swerve swerve) {
		return swerve.run(() -> swerve.drive(new Translation2d(-0.5, 0), 0.0, true, kSwerve.openLoop))
				.withTimeout(
						2.5);
	}

}
