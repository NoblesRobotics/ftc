package org.firstinspires.ftc.teamcode.nobles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private TrajectoryDrive drive;
    private AprilTagFinder aprilTagFinder;
    private DistanceSensor distanceSensor;

    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        drive = new TrajectoryDrive(hardwareMap);
        aprilTagFinder = new AprilTagFinder(hardwareMap);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "da");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.followTrajectory(drive.standardCollect);
            } else if (gamepad1.b) {
                int targetCodeOffset = 0;
                if (gamepad1.dpad_left) targetCodeOffset = -1;
                else if (gamepad1.dpad_right) targetCodeOffset = 1;

                drive.followTrajectory(drive.standardPlace);
                alignWithBackdropCenter(targetCodeOffset);
                approachBackdrop();
            } else if (gamepad1.x) {
                drive.turnToZeroHeading();
            }
            drive.followGamepad(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }

    private void approachBackdrop() {
        telemetry.addData("backX", drive.getPoseEstimate().getX());
        telemetry.addData("backY", drive.getPoseEstimate().getY());
        telemetry.update();

        double approachDistance = distanceSensor.getDistance(DistanceUnit.INCH) - 6;
        drive.followCustomTrajectory(approachDistance, 0);
        drive.setPoseEstimate(new Pose2d(0, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
    }

    private void alignWithBackdropCenter(int targetCodeOffset) {
        boolean didAlign = false;
        double lateralError = 9999;
        for (int alignmentAttempt = 0; alignmentAttempt < 5 && Math.abs(lateralError) > 0.1; alignmentAttempt++) {
            lateralError = 9999;
            for (int locationAttempt = 0; locationAttempt < 10 && lateralError == 9999; locationAttempt++) {
                lateralError = getBackdropLateralError(targetCodeOffset);
                sleep(50);
            }

            TelemetryStatic.telemetry.addData("lateral" + alignmentAttempt, lateralError);
            if (lateralError != 9999) {
                drive.followCustomTrajectory(0, -lateralError);
                didAlign = true;
            }
            else break;
        }

        if (didAlign) drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), -6 * targetCodeOffset, drive.getPoseEstimate().getHeading()));
    }

    private double getBackdropLateralError(int targetCodeOffset) {
        double distanceBetweenCodes = 6;

        AprilTagFinder.TransformedDetection detection = aprilTagFinder.getDetection(5, telemetry);
        if (detection != null) return detection.x + distanceBetweenCodes * targetCodeOffset;

        detection = aprilTagFinder.getDetection(4, telemetry);
        if (detection != null) return detection.x + distanceBetweenCodes * (targetCodeOffset + 1);

        detection = aprilTagFinder.getDetection(6, telemetry);
        if (detection != null) return detection.x + distanceBetweenCodes * (targetCodeOffset - 1);

        return 9999;
    }
}
