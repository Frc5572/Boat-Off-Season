package frc.robot.Auto;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class autoTest extends SequentialCommandGroup {
    Drivetrain drivetrain;
    Trajectory trajectory = new Trajectory();

    public autoTest(Drivetrain drivetrain) {
        // PathPlannerTrajectory autoTest = PathPlanner.loadPath("autoTest", 6, 3);
        // PPMecanumControllerCommand command = baseMecanumCommand(autoTest);
        // PathPlannerState intialstate = autoTest.getInitialState();
        // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        // new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
        // Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
        // Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
        // Constants.DrivetrainConstants.kDriveKinematics, 10);

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath()
                .resolve("pathplanner/generatedJSON/autoTest.wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {

        }

        RamseteCommand ramseteCommand = new RamseteCommand(trajectory, drivetrain::getPose,
            new RamseteController(Constants.DrivetrainConstants.kRamseteB,
                Constants.DrivetrainConstants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.DrivetrainConstants.ksVolts,
                Constants.DrivetrainConstants.kvVoltSecondsPerMeter,
                Constants.DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DrivetrainConstants.kDriveKinematics, drivetrain::getWheelSpeeds,
            new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DrivetrainConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts, drivetrain);


        addCommands(new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            ramseteCommand, new InstantCommand(() -> drivetrain.tankDriveVolts(0, 0)));
    }
}
