package org.firstinspires.ftc.teamcode.nobles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectoryDrive {
    public final Trajectory standardCollect, standardPlace;

    private final MecanumDriveImpl drive;

    public TrajectoryDrive(HardwareMap hardwareMap) {
        drive = new MecanumDriveImpl(hardwareMap);
        drive.setPoseEstimate(new Pose2d());

        standardCollect = drive.trajectoryBuilder(new Pose2d(), true)
                .splineTo(scaleVector(-72, 0), Math.toRadians(180))
                .splineToConstantHeading(scaleVector(-114, 96), Math.toRadians(180))
                .build();
        standardPlace = drive.trajectoryBuilder(standardCollect.end())
                .splineToConstantHeading(scaleVector(-108, 0), Math.toRadians(0))
                .splineTo(scaleVector(-10, 0), Math.toRadians(0))
                .build();
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void followCustomTrajectory(double driveDistance, double strafeDistance) {
        Pose2d initialPose = drive.getPoseEstimate();
        drive.setPoseEstimate(new Pose2d(0, 0, drive.getPoseEstimate().getHeading()));

        Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(driveDistance, strafeDistance))
                .build();
        drive.followTrajectory(trajectory);

        drive.setPoseEstimate(new Pose2d(
                initialPose.getX() + drive.getPoseEstimate().getX(),
                initialPose.getY() + drive.getPoseEstimate().getY(),
                drive.getPoseEstimate().getHeading()
        )); // BIG CHANGE
    }

    public void turnToZeroHeading() {
        TelemetryStatic.telemetry.addData("heading_before", drive.getPoseEstimate().getHeading());
        TelemetryStatic.telemetry.update();
        drive.turn(-drive.getPoseEstimate().getHeading());
        TelemetryStatic.telemetry.addData("heading_after", drive.getPoseEstimate().getHeading());
        TelemetryStatic.telemetry.update();
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void followGamepad(double drivePower, double strafePower, double turnPower) {
        if (drivePower <= 0.1) drivePower = 0;
        if (strafePower <= 0.1) strafePower = 0;
        if (turnPower <= 0.1) turnPower = 0;
        drive.setMotorPowers(
                drivePower + strafePower - turnPower, // leftFront
                drivePower - strafePower - turnPower, // leftRear
                drivePower - strafePower + turnPower, // rightRear
                drivePower + strafePower + turnPower // rightFront
        );
    }

    private Vector2d scaleVector(double x, double y) {
        double scaleConstant = 60. / 64;
        return new Vector2d(x * scaleConstant, y * scaleConstant);
    }
}
