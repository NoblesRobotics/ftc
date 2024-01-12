package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

@TeleOp
public class SwerveTrajectory extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        SwerveDriveImpl drive = new SwerveDriveImpl(hardwareMap);

        while (!isStarted()) drive.calibrateServos();
        waitForStart();

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(), 0)
                        .strafeLeft(18)
                        .build()
        );
    }
}