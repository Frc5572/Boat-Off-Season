package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Constants for drivetrain
     */
    public static final class DrivetrainConstants {
        public static final double trackWidthMeters = Units.inchesToMeters(0);
        public static final int FRONTLEFTMOTORID = 2;
        public static final int FRONTRIGHTMOTORID = 1;
        public static final int BACKLEFTMOTORID = 3;
        public static final int BACKRIGHTMOTORID = 4;
        public static final int ksVolts = 0;
        public static final int kvVoltSecondsPerMeter = 0;
        public static final int kaVoltSecondsSquaredPerMeter = 0;
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;
        public static final int kPDriveVel = 1;
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(trackWidthMeters);
        public static final boolean invertGyro = false;
    }

    /**
     * Constants for the Pnuematic Systems
     */
    public static final class PneumaticsConstants {
        public static final int FORWARDCHANNEL = 3;
        public static final int REVERSECHANNEL = 4;
    }

}
