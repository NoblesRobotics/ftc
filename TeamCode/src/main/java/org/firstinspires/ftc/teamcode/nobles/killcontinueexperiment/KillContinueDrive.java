package org.firstinspires.ftc.teamcode.nobles.killcontinueexperiment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

public class KillContinueDrive {
    private final MecanumDriveImpl drive;
    private final Notifiable notifiable = new Notifiable();

    private ATTrajectory currentTrajectory;
    private int currentSegmentIndex;
    private boolean killed = false;
    private MarkerCallback onEndTrajectory = null;

    public KillContinueDrive(MecanumDriveImpl drive) {
        this.drive = drive;
    }

    public void followTrajectory(ATTrajectory trajectory, MarkerCallback onEndTrajectory) {
        if (currentTrajectory != null && !killed) return;

        currentTrajectory = trajectory;
        currentSegmentIndex = 0;
        killed = false;
        this.onEndTrajectory = onEndTrajectory;
        trajectory.setNotifiable(notifiable);
        drive.followTrajectoryAsync(trajectory.trajectory);
    }

    public void followTrajectory(ATTrajectory trajectory) {
        if (currentTrajectory != null && !killed) return;

        followTrajectory(trajectory, null);
    }

    public void update() {
        if (currentTrajectory != null && !killed) drive.update();
    }

    public void killTrajectory() {
        if (currentTrajectory == null || killed) return;

        killed = true;
        drive.setMotorPowers(0, 0, 0, 0);
        // sleep?
    }

    public void continueTrajectory() {
        if (currentTrajectory == null || !killed) return;

        Pose2d[] poses = new Pose2d[currentTrajectory.poses.length - currentSegmentIndex];
        TelemetryStatic.telemetry.addData("posesLength", currentTrajectory.poses.length); // 3
        TelemetryStatic.telemetry.addData("segmentIndex", currentSegmentIndex); // 0
        TelemetryStatic.telemetry.update();
        poses[0] = drive.getPoseEstimate();
        for (int i = 1, j = currentSegmentIndex + 1; i < poses.length; i++, j++) {
            poses[i] = currentTrajectory.poses[j];
        }

        ATTrajectory restoredTrajectory = new ATTrajectory(drive, poses, currentTrajectory.startHeading);
        followTrajectory(restoredTrajectory, onEndTrajectory);
    }

    public void eraseTrajectory() {
        if (currentTrajectory == null || !killed) return;

        currentTrajectory = null;
    }

    class Notifiable implements ATNotifiable {
        public void onNextSegment() {
            currentSegmentIndex++;
        }

        public void onEndTrajectory() {
            currentTrajectory = null;
            if (onEndTrajectory != null) onEndTrajectory.onMarkerReached();
        }
    }
}
