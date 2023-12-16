package org.firstinspires.ftc.teamcode.nobles.killcontinueexperiment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

public class ATTrajectory {
    public final Trajectory trajectory;
    public final Pose2d[] poses;
    public final double startHeading;
    private ATNotifiable notifiable;

    public ATTrajectory(MecanumDriveImpl drive, Pose2d[] poses, double startHeading) {
        this.poses = poses;
        this.startHeading = startHeading;

        TrajectoryBuilder builder = drive.trajectoryBuilder(poses[0], startHeading);
        TelemetryStatic.telemetry.addData("start", startHeading);
        for (int i = 1; i < poses.length; i++) {
            builder
                    .splineToConstantHeading(
                        scaleVector(poses[i].getX(), poses[i].getY()),
                        poses[i].getHeading()
                    )
                    .addDisplacementMarker(() -> {if (notifiable != null) notifiable.onNextSegment();});
            TelemetryStatic.telemetry.addData("head" + i, poses[i].getHeading());
        }
        TelemetryStatic.telemetry.update();
        trajectory = builder
                .addDisplacementMarker(() -> {if (notifiable != null) notifiable.onEndTrajectory();})
                .build();
    }

    public void setNotifiable(ATNotifiable notifiable) {
        this.notifiable = notifiable;
    }

    private Vector2d scaleVector(double x, double y) {
        double scaleConstant = 60. / 64;
        return new Vector2d(x * scaleConstant, y * scaleConstant);
    }
}
