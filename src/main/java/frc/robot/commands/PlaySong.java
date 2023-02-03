package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PlaySong extends CommandBase {
	private Orchestra player = new Orchestra();

	public PlaySong(Swerve swerve, String file) {
		for (SwerveModule mod : swerve.mSwerveMods) {
			player.addInstrument(mod.mAngleMotor);
			player.addInstrument(mod.mDriveMotor);
		}

		player.loadMusic(file);

		player.play();

		addRequirements(swerve);
	}

	public void end(boolean interrupted) {
		player.stop();
	}

	public boolean isFinished() {
		return this.player.isPlaying();
	}
}
