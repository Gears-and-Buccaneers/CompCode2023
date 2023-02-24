package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Gripper extends SubsystemBase {
	public final DoubleSolenoid gripper = new DoubleSolenoid(Constants.Pneumatics.compressorId,
			PneumaticsModuleType.CTREPCM,
			Constants.Gripper.forwardId, Constants.Gripper.reverseId);

	public void open() {
		gripper.set(Value.kReverse);
	}

	public void close() {
		gripper.set(Value.kForward);
	}

	public void toggle() {
		gripper.toggle();
	}

	public boolean isOpen() {
		return (gripper.get().equals(Value.kForward));
	}
}
