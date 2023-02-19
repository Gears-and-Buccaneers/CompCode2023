package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Constants;

public class Arm extends SubsystemBase {
	public TalonSRX controller = new TalonSRX(Constants.armControllerId);

	public Arm() {
		controller.setNeutralMode(NeutralMode.Brake);
	}
}
