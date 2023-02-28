package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
	PIDController pid = new PIDController(0.5, 0, .02);
	Encoder encoder = new Encoder(2, 3);

	DoubleSolenoid pneumatic = new DoubleSolenoid(
			Pneumatics.compressorId, PneumaticsModuleType.CTREPCM,
			BoomC.forwardId, BoomC.reverseId);

	boolean raised = pneumatic.get() == Value.kForward;

	Servo rachet = new Servo(BoomC.servoId);

	public Boom() {
		controller.setNeutralMode(NeutralMode.Brake);
		pid.setTolerance(0.5);
	}

	public CommandBase setTo(Level level) {
		CommandBase l = this.runOnce(() -> pid.setSetpoint(level.getLength()))
				.andThen(new WaitUntilCommand(pid::atSetpoint));

		// if (raised == level.isRaised())
		// return l;

		CommandBase r = runOnce(() -> {
			System.out.println("r: setting the pneumatic to " + level.isRaised());
			pneumatic.set(level.isRaised() ? Value.kForward : Value.kReverse);
		}).andThen(
				new WaitCommand(BoomC.raiseDelay),
				runOnce(() -> raised = level.isRaised()));

		return level.isRaised() ? r.andThen(l) : l.andThen(r);
	}

	public double getEncoder() {
		return (double) encoder.get() / 2048.0;
	}

	public void periodic() {
		double output = -pid.calculate(getEncoder());
		output = Math.abs(output) > 0.5 ? output / Math.abs(output) * 0.5 : output;
		controller.set(ControlMode.PercentOutput, output);

		rachet.set(SmartDashboard.getNumber("Boom servo", 0));

		SmartDashboard.putBoolean("Boom raised", raised);
		SmartDashboard.putBoolean("Boom within tolerance", pid.atSetpoint());
		SmartDashboard.putNumber("Boom output", output);
		SmartDashboard.putNumber("Boom Encoder", getEncoder());
	}

}
