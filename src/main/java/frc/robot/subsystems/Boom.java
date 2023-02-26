package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import frc.robot.Constants.BoomC;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.BoomC.Level;

public class Boom extends SubsystemBase {
	TalonSRX controller = new TalonSRX(BoomC.controllerId);
	PIDController pid = new PIDController(0.5, 0, .01);
	Encoder encoder = new Encoder(2, 3);

	Solenoid pneumatic = new Solenoid(
			Pneumatics.compressorId, PneumaticsModuleType.CTREPCM,
			BoomC.id);
	boolean raised = pneumatic.get();

	Servo rachet = new Servo(BoomC.servoId);

	public Boom() {
		controller.setNeutralMode(NeutralMode.Brake);
		pid.setTolerance(0.2);
	}

	public CommandBase setTo(Level level) {
		CommandBase l = this.runOnce(() -> pid.setSetpoint(level.getLength()))
				.andThen(new WaitUntilCommand(pid::atSetpoint));

		if (raised == level.isRaised())
			return l;

		CommandBase r = runOnce(() -> pneumatic.set(level.isRaised())).andThen(new WaitCommand(BoomC.raiseDelay),
				runOnce(() -> raised = level.isRaised()));

		return level.isRaised() ? r.andThen(l) : l.andThen(r);
	}

	public double getEncoder() {
		return encoder.get() / 2048;
	}

	public void periodic() {
		double output = -pid.calculate(encoder.get() / 2048);
		controller.set(ControlMode.PercentOutput, output * 0.01);

		SmartDashboard.putNumber("Boom output", output);
		SmartDashboard.putNumber("Boom Encoder", encoder.get());
	}
}
