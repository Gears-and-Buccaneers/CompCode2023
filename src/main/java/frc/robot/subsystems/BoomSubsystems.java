package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.configs.Constants;
import frc.lib.configs.Constants.Subsystems.Boom;
public class BoomSubsystems extends SubsystemBase {
	public TalonSRX controller = new TalonSRX(Constants.armControllerId);
	public PIDController pid = new PIDController(.1,0,0);
	public Encoder encoder = new Encoder(2,3);
	public BoomSubsystems() {
		controller.setNeutralMode(NeutralMode.Brake);
		encoder.setDistancePerPulse(1/2048);
	}

	public void set(Double rots) {
		SmartDashboard.putNumber("bore Encoder", encoder.get());
		double sett = pid.calculate(encoder.get(), rots);
		
		SmartDashboard.putNumber("pid", sett);
		controller.set(ControlMode.PercentOutput, sett);
	}
	public void exstendBottomRow() {
		set(Boom.BottomRowRots);
	}
	public void exstendMiddleRow() {
		set(Boom.MiddleRowRots);
	}
	public void exstendTopRow() {
		set(Boom.TopRowRots);
	}
	public void exstendIntakeSpot() {
		set(Boom.IntakeRots);
	}
}
