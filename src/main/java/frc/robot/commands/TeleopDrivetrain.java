// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class TeleopDrivetrain extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;
    private final XboxController driver;

    /**
     * Creates a new ExampleCommand.
     *
     * @param drivetrain The drive train subsystem used by this command.
     * @param driver Driver Xbox Controller
     */
    public TeleopDrivetrain(Drivetrain drivetrain, XboxController driver) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (drivetrain.mecanumState) {
            drivetrain.driveTank(driver.getLeftY() / 2, driver.getRightY() / 2);
        } else {
            drivetrain.driveMecanum(driver.getLeftY() / 2, -driver.getLeftX() / 2,
                -driver.getRightX() / 2);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
