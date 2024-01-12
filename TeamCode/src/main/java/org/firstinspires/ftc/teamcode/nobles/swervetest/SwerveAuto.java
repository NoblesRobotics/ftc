package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.nobles.slidetest.SlideAssembly;

@TeleOp
public class SwerveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        drive.calibrateServos();

        waitForStart();

        drive.setAngle(0);
        //drive.drive(48);
        drive.drive(30);
        drive.turn(90);
        assembly.setIntakePower(-1);
        sleep(1000);
        assembly.setIntakePower(0);
        drive.setAngle(-90);
        drive.drive(24);
        drive.setAngle(0);
    }
}
