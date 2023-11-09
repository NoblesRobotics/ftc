package org.firstinspires.ftc.teamcode.nobles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    private AprilTagFinder aprilTagFinder;

    public void runOpMode() {
        TrajectoryDrive drive = new TrajectoryDrive(hardwareMap);
        aprilTagFinder = new AprilTagFinder(hardwareMap);
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "da");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.followTrajectory(drive.standardCollect);
            } else if (gamepad1.x) {
                drive.followTrajectory(drive.standardPlace);
            } else if (gamepad1.b) {
                double lateralError = 0;
                for ( int i = 0; i < 10 && lateralError == 0; i++ ) {
                    lateralError = getBackdropLateralError();
                    sleep(100);
                }
                telemetry.addData("lateral", lateralError);
                telemetry.update();
                drive.followCustomTrajectory(0, -lateralError);
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, drive.getPoseEstimate().getHeading()));
            }
            drive.followGamepad(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private double getBackdropLateralError() {
        double distanceBetweenCodes = 6;

        AprilTagFinder.TransformedDetection detection = aprilTagFinder.getDetection(5, telemetry);
        if (detection != null) return detection.x;

        detection = aprilTagFinder.getDetection(4, telemetry);
        if (detection != null) return detection.x + distanceBetweenCodes;

        detection = aprilTagFinder.getDetection(6, telemetry);
        if (detection != null) return detection.x - distanceBetweenCodes;

        return 0;
    }
}
