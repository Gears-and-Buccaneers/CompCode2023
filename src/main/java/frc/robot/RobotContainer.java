package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Constants
import frc.lib.configs.OIConstants.*;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /**Controlers*/
  private final Joystick driver = new Joystick(Driver.kPort);
  private final Driver driverButtons = new Driver(driver);

  private final Joystick operator = new Joystick(Operator.kPort);
  private final Driver operatorButtons = new Driver(operator);
    


  /*subsytems -- for subsytems name them (name)Subsytem and when defining them here givethm s_(name)*/
  private final Swerve s_Swerve = new Swerve();

  /* Autos */
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private final Command m_simpleAuto = null;
  private final Command m_complexAuto = new exampleAuto(s_Swerve);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, Driver.kYAxis, Driver.kXAxis, Driver.kRotAxis, true, true));

    // Configure the button bindings
    configureButtonBindings();

    //setup Autos
    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);
    m_chooser.addOption("Complex Auto", m_complexAuto);
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverButtons.Back.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
