package frc.robot;

public class Constants {
    public static class DriveConstants {
        public static int[] swerveMotorDrivePorts = {2, 5, 8, 11}; //TODO: update port numbers
        public static int[] swerveMotorTurnPorts = {1, 4, 7, 10}; //TODO: update port numbers
        public static int[] swerveEncoderPorts = {3, 6, 9, 12}; //TODO: update port numbers

        public static double[] turnAngleOffsets = {-26.0695796785, 40.9664823519,-82.3340351603, 17.0168465154}; //degrees
        public static boolean[] turnEncoderInversed = {false, false, false, false};

        public static double turnPID_P = 0.5;
        public static double turnPID_I = 0;
        public static double turnPID_D = 0;
    }
}
