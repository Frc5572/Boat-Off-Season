// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.rotate;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final XboxController driver = new XboxController(0);
    // private final XboxController operator = new XboxController(1);
    // private final Drivetrain drivetrain = new Drivetrain();

    private final Encoder encoder = new Encoder(1, 2);
    private final DutyCycleEncoder dEncoder = new DutyCycleEncoder(0);
    private final CANSparkMax max = new CANSparkMax(5, MotorType.kBrushless);

    private final double encOffset = 0.949;

    private double encVal() {
        return -(dEncoder.getAbsolutePosition() - encOffset);
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // default command constantly runs unless interrupted by another command.
        // drivetrain.setDefaultCommand(new TeleopDrivetrain(drivetrain, driver));
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // While A is pressed, create a new start end command that moves the intake in when
        // initialized and
        // stops the motor when the command ends (button is no longer pressed).
        // new JoystickButton(driver, XboxController.Button.kA.value)
        // .whenPressed(new InstantCommand(() -> {
        // if (drivetrain.mecanumState) {
        // drivetrain.mecanum.set(Value.kForward);
        // } else {
        // drivetrain.mecanum.set(Value.kReverse);
        // }
        // drivetrain.mecanumState = !drivetrain.mecanumState;
        // }));

        new JoystickButton(driver, XboxController.Button.kB.value)
            .onTrue(new InstantCommand(() -> System.out
                .println(".get Relative: " + encoder.get() + " .get ABS: " + (encVal()))));
        new JoystickButton(driver, XboxController.Button.kA.value)
            .onTrue(new rotate(max, dEncoder, 100));

        new JoystickButton(driver, XboxController.Button.kX.value)
            .onTrue(new InstantCommand(() -> max.set(.1)));
        // same thing except with button b and will move the intake out instead of in.
        // new JoystickButton(operator, XboxController.Button.kB.value).whileHeld(new
        // StartEndCommand(() -> intake.Out(), () -> intake.Stop(), intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
