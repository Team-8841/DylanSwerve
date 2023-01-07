package frc.robot;

public class Constants {
    public static class DriveConstants {
        public static int[] swerveMotorDrivePorts = {1, 4, 7, 10}; //TODO: update port numbers
        public static int[] swerveMotorTurnPorts = {2, 5, 8, 11}; //TODO: update port numbers
        public static int[] swerveEncoderPorts = {3, 6, 9, 12}; //TODO: update port numbers

        public static float[] turnAngleOffsets = {0, 0, 0, 0};

        public static double turnPID_P = 0.5;
        public static double turnPID_I = 0;
        public static double turnPID_D = 0;
    }
}
