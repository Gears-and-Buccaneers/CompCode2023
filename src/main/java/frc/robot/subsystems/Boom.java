package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Boom.Level;

public class Boom extends SubsystemBase {
	public TalonSRX controller = new TalonSRX(Constants.Boom.controllerId);
	public PIDController pid = new PIDController(0.5, 0, .01);
	public Encoder encoder = new Encoder(2, 3);

	public final Solenoid pneumatic = new Solenoid(
			Constants.Pneumatics.compressorId, PneumaticsModuleType.CTREPCM,
			Constants.Boom.id);

	private boolean raised = pneumatic.get();
	private double targetLength = encoder.get() / 2048, lastOutput;

	public Boom() {
		controller.setNeutralMode(NeutralMode.Brake);
		pid.setTolerance(0.2);
	}

	public CommandBase setTo(Level level) {
		WaitUntilCommand waitLength = new WaitUntilCommand(() -> lastOutput < Constants.Boom.pidDeadband);
		waitLength.addRequirements(this);
		CommandBase l = this.runOnce(() -> targetLength = level.getLength())
				.andThen(waitLength);

		targetLength = level.getLength();
		pollPID();

		if (raised == level.isRaised())
			return l;

		CommandBase waitRaised = new WaitCommand(Constants.Boom.raiseDelay);
		waitRaised.addRequirements(this);
		waitRaised = waitRaised.andThen(runOnce(() -> raised = level.isRaised()));

		CommandBase r = runOnce(() -> pneumatic.set(level.isRaised()));

		return level.isRaised() ? r.andThen(l) : l.andThen(r);
	}

	public void pollPID() {
		double output = -pid.calculate(encoder.get() / 2048, targetLength);
		controller.set(ControlMode.PercentOutput, output);

		SmartDashboard.putNumber("Boom output", output);
		SmartDashboard.putNumber("Boom Encoder", encoder.get());
	}

	public void periodic() {
		pollPID();
	}

	public boolean isRaised() {
		return pneumatic.get();
	}
}
