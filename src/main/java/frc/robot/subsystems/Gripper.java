package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kGripper;
import frc.robot.Constants.kPneumatics;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Gripper extends SubsystemBase {
	public final DoubleSolenoid gripper = new DoubleSolenoid(kPneumatics.compressorId,
			PneumaticsModuleType.CTREPCM,
			kGripper.forwardId, kGripper.reverseId);

	public void toggle() {
		if (gripper.get() == Value.kForward)
			gripper.set(Value.kReverse);
		else
			gripper.set(Value.kForward);
	}
}
