package org.firstinspires.ftc.teamcode.swerve.servo;

public class SwerveServoStorage {
    // In the FTC system, static class variables are preserved across different opmodes being run
    // (e.g. when switching from auto to teleop) but not across a robot power cycle

    public static double[] refPositions = {0, 0, 0, 0};
    public static int[] currentRevs = {0, 0, 0, 0};
    public static boolean hasCachedPositions = false;

    private SwerveServoStorage() {}
}
