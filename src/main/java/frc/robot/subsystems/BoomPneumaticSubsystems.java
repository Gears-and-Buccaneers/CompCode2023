package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.PneumaticsModuleType;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.configs.Constants.Phematics;
import frc.lib.configs.Constants.Subsystems.Boom;
public class BoomPneumaticSubsystems extends SubsystemBase {
    public final DoubleSolenoid gripper = new DoubleSolenoid(Phematics.compressorId,PneumaticsModuleType.CTREPCM, Boom.forwardId, Boom.reverseId);
    /** Creates a new ExampleSubsystem. */
    public BoomPneumaticSubsystems() {
        
    }

    /**@return exsteds the phematic */
    public void exstend() {
        gripper.set(Value.kReverse);
    }

    /**@return retacts the phematic */
    public void retract() {
        gripper.set(Value.kForward);
    }

    /**@return toogles the state of the phematic */
    public void toggle() {
        gripper.toggle();
    }

    /**@return true if state is forward and false if refersed or off */
    public boolean getState() {
        return (gripper.get().equals(Value.kForward));
    }
}
