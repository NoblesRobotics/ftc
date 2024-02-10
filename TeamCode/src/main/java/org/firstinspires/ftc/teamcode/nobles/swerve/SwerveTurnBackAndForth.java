package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;

@TeleOp
public class SwerveTurnBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 60, 8.75, 1);
        drive.calibrateServos();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            drive.turn(90);
            drive.turn(0);
        }
    }
}
