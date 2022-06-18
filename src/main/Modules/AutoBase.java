package frc.robot.Modules;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoBase extends SequentialCommandGroup {
    public Drivetrain drivetrain;

    public PPMecanumControllerCommand baseSwerveCommand(PathPlannerTrajectory trajectory) {
        PPMecanumControllerCommand command = new PPMecanumControllerCommand(trajectory,
            drivetrain::getPose, Constants.drivetrain.Kinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0), thetaController,
            drivetrain::setModuleStates, drivetrain);
        return command;
}
