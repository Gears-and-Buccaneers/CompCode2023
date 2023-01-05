// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.configs;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class OIConstants {
    public static final class Driver {
        public static final int kPort = 0;
    
        public static final int kYAxis = 1;
        public static final int kXAxis = 0;
        public static final int kRotAxis = 4;

        public final JoystickButton A;
        public final JoystickButton B;
        public final JoystickButton X;
        public final JoystickButton Y;
        public final JoystickButton LB;
        public final JoystickButton RB;
        public final JoystickButton Back;
        public final JoystickButton start;
        public final JoystickButton LeftThumb;
        public final JoystickButton RightThumb;

        public Driver(Joystick joystick) {
            A = new JoystickButton(joystick, 1);
            B = new JoystickButton(joystick, 2);
            X = new JoystickButton(joystick, 3);
            Y = new JoystickButton(joystick, 4);
            LB = new JoystickButton(joystick, 5);
            RB = new JoystickButton(joystick, 6);
            Back = new JoystickButton(joystick, 7);
            start = new JoystickButton(joystick, 8);
            LeftThumb = new JoystickButton(joystick, 9);
            RightThumb = new JoystickButton(joystick, 10);
        }
    }
    public static final class Operator {
        public static final int kPort = 1;

        public final JoystickButton A;
        public final JoystickButton B;
        public final JoystickButton X;
        public final JoystickButton Y;
        public final JoystickButton LB;
        public final JoystickButton RB;
        public final JoystickButton Back;
        public final JoystickButton start;
        public final JoystickButton LeftThumb;
        public final JoystickButton RightThumb;

        public Operator(Joystick joystick) {
            A = new JoystickButton(joystick, 1);
            B = new JoystickButton(joystick, 2);
            X = new JoystickButton(joystick, 3);
            Y = new JoystickButton(joystick, 4);
            LB = new JoystickButton(joystick, 5);
            RB = new JoystickButton(joystick, 6);
            Back = new JoystickButton(joystick, 7);
            start = new JoystickButton(joystick, 8);
            LeftThumb = new JoystickButton(joystick, 9);
            RightThumb = new JoystickButton(joystick, 10);
        }
    }
    public static final double kDeadband = 0.05;
}