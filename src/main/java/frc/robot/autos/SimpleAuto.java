package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kBoom.Level;
import frc.robot.subsystems.Boom;
import frc.robot.subsystems.Gripper;

public class SimpleAuto {
	public CommandBase create(Boom arm, Gripper gripper) {
		return arm.setTo(Level.INTAKE).andThen(arm.setTo(Level.TOP), gripper.runOnce(gripper::toggle),
				new WaitCommand(0), gripper.runOnce(gripper::toggle));
	}
}
