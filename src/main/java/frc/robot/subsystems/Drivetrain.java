// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Drivetrain Subsystem
 */
public class Drivetrain extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    MotorController m_frontLeft;
    MotorController m_rearLeft;
    MotorControllerGroup m_left;
    MotorController m_frontRight;
    MotorController m_rearRight;
    MotorControllerGroup m_right;
    DifferentialDrive tankDrive;
    MecanumDrive mecanumDrive;
    public DoubleSolenoid mecanum;
    public boolean mecanumState = false;
    public DifferentialDriveOdometry differentialDriveOdometry;
    public AHRS gyro;

    /**
     * Drivetrain Subsystem
     */
    public Drivetrain() {
        gyro = new AHRS(Port.kMXP);
        differentialDriveOdometry = new DifferentialDriveOdometry(getYaw());
        RelativeEncoder encoder =
            m_frontLeft.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        mecanum = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.PneumaticsConstants.FORWARDCHANNEL,
            Constants.PneumaticsConstants.REVERSECHANNEL);
        m_frontLeft =
            new CANSparkMax(Constants.DrivetrainConstants.FRONTLEFTMOTORID, MotorType.kBrushless);
        m_rearLeft =
            new CANSparkMax(Constants.DrivetrainConstants.BACKLEFTMOTORID, MotorType.kBrushless);
        m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);

        m_frontRight =
            new CANSparkMax(Constants.DrivetrainConstants.FRONTRIGHTMOTORID, MotorType.kBrushless);
        m_rearRight =
            new CANSparkMax(Constants.DrivetrainConstants.BACKRIGHTMOTORID, MotorType.kBrushless);
        m_right = new MotorControllerGroup(m_frontRight, m_rearRight);

        // m_right.setInverted(true);
        m_rearRight.setInverted(true);
        m_frontRight.setInverted(true);

        mecanum.set(Value.kForward);

        // tankDrive = new DifferentialDrive(m_left, m_right);
        mecanumDrive = new MecanumDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight);
    }

    public void zeroGyro() {
        gyro.zeroYaw();
    }

    /**
     * Gets the rotation degree from swerve modules.
     */
    public Rotation2d getYaw() {
        float yaw = gyro.getYaw();
        return (Constants.DrivetrainConstants.invertGyro) ? Rotation2d.fromDegrees(360 - yaw)
            : Rotation2d.fromDegrees(yaw);
    }


    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Default drive function
     *
     * @param yaxis Y-Axis power
     * @param xaxis X-Axis Power
     * @param rotation Rotation Power
     */
    public void drive(double yaxis, double xaxis, double rotation) {
        // if(mecanumState) {
        // tankDrive.tankDrive(left, right);
        // } else {
        mecanumDrive.driveCartesian(yaxis, xaxis, rotation);
        // }
    }

    public Pose2d getPose() {
        return differentialDriveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds();
    }

    public void tankDriveVolts(double number, double number2) {

    }

    public void resetOdometry(Pose2d pose) {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
