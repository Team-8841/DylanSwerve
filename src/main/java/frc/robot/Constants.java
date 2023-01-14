package frc.robot;

public class Constants {

    public static class DriveConstants {

        public static int[] swerveMotorDrivePorts = {2, 5, 8, 11}; //TODO: update port numbers
        public static int[] swerveMotorTurnPorts = {1, 4, 7, 10}; //TODO: update port numbers
        public static int[] swerveEncoderPorts = {3, 6, 9, 12}; //TODO: update port numbers

        public static final int kCurrentLimit = 60; // default for neo is 80
        public static final double kDTRampRate = 0.1; // Drivetrain open loop ramp rate |
                                                              // tune to slow
                                                              // acceleration (higher = slower)

        public static double[] turnAngleOffsets = {90, 90, 90, 90}; //degrees
        public static boolean[] turnEncoderInversed = {false, false, false, false};

        public static double turnPID_P = 0.5;
        public static double turnPID_I = 0;
        public static double turnPID_D = 0;

        public static double driveDeadband = 0.05;
    }

}
