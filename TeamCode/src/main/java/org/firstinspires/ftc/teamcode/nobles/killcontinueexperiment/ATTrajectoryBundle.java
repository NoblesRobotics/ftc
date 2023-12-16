package org.firstinspires.ftc.teamcode.nobles.killcontinueexperiment;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl;

public class ATTrajectoryBundle {
    public static ATTrajectory standardCollect, standardPlace;

    public static void create(MecanumDriveImpl drive) {
        standardCollect = new ATTrajectory(
                drive,
                new Pose2d[] {
                        new Pose2d(0, 0, Math.toRadians(0)),
                        new Pose2d(-72, 0, Math.toRadians(180)),
                        new Pose2d(-114, 96, Math.toRadians(180))
                },
                Math.toRadians(180)
        );
        standardPlace = new ATTrajectory(
                drive,
                new Pose2d[] {
                        new Pose2d(-114, 96, Math.toRadians(0)),
                        new Pose2d(-108, 0, Math.toRadians(0)),
                        new Pose2d(-10, 0, Math.toRadians(0))
                },
                Math.toRadians(0)
        );
    }
}
