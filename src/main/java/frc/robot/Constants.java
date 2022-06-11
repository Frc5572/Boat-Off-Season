package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // constants for drivetrain
    public static final class DrivetrainConstants {
        public static final int FRONTLEFTMOTORID = 2;
        public static final int FRONTRIGHTMOTORID = 1;
        public static final int BACKLEFTMOTORID = 3;
        public static final int BACKRIGHTMOTORID = 4;
    }
    // constants for the arm
    public static final class PneumaticsConstants {
        public static final int FORWARDCHANNEL = 3;
        public static final int REVERSECHANNEL = 4;
    }
}
