package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

@TeleOp
public class SwerveTurnBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        drive.calibrateServos();

        waitForStart();

        while (opModeIsActive()) {
            drive.turn(90);
            sleep(3000);
            drive.turn(0);
            sleep(3000);
        }
    }
}
