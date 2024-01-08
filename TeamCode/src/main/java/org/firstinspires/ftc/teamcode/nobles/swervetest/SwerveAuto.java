package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;

@TeleOp
public class SwerveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        drive.calibrateServos();

        waitForStart();

        drive.setAngle(0);
        drive.drive(48);
    }
}
