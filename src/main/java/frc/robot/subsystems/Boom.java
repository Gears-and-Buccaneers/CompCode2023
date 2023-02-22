package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Constants;
import frc.lib.configs.Constants.Boom.BoomLevel;

public class Boom extends SubsystemBase {
	public TalonSRX controller = new TalonSRX(Constants.armControllerId);
	public PIDController pid = new PIDController(0.5, 0, .01);
	public Encoder encoder = new Encoder(2, 3);

	public final DoubleSolenoid pneumatic = new DoubleSolenoid(
			Constants.Pneumatics.compressorId, PneumaticsModuleType.CTREPCM,
			Constants.Boom.forwardId, Constants.Boom.reverseId);

	public Boom() {
		controller.setNeutralMode(NeutralMode.Brake);

		pid.setTolerance(0.2);
	}

	public CommandBase extendTo(BoomLevel level) {
		return runEnd(() -> {
			double output = pid.calculate(encoder.get() / 2048, level.getLength());
			// output /= 2048;
			SmartDashboard.putNumber("Boom output", -output);

			controller.set(ControlMode.PercentOutput, -output);
		}, () -> controller.set(ControlMode.PercentOutput, 0));
	}

	public void periodic() {
		SmartDashboard.putNumber("Boom Encoder", encoder.get());
	}

	/** @return extends the pneumatic */
	public void raise() {
		pneumatic.set(Value.kForward);
	}

	/** @return retracts the pneumatic */
	public void lower() {
		pneumatic.set(Value.kReverse);
	}

	/** @return toogles the state of the pneumatic */
	public void toggle() {
		pneumatic.toggle();
	}

	public boolean pneumaticExtended() {
		return (pneumatic.get().equals(Value.kForward));
	}
}
