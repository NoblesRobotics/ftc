package org.firstinspires.ftc.teamcode.nobles.killcontinueexperiment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

@TeleOp
public class KCTeleOp extends LinearOpMode {
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;

        MecanumDriveImpl drive = new MecanumDriveImpl(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        KillContinueDrive kcDrive = new KillContinueDrive(drive);
        ATTrajectoryBundle.create(drive);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                kcDrive.followTrajectory(ATTrajectoryBundle.standardCollect);
            } else if (gamepad1.b) {
                kcDrive.followTrajectory(ATTrajectoryBundle.standardPlace);
            } else if (gamepad1.left_bumper) {
                kcDrive.killTrajectory();
            } else if (gamepad1.right_bumper) {
                kcDrive.continueTrajectory();
            }
            kcDrive.update();
        }
    }
}
