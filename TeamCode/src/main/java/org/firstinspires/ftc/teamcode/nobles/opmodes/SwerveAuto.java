package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.AutoSwerveDrive;

@TeleOp
public class SwerveAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;
        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        drive.calibrateServos();
        //BlockFinder blockFinder = new BlockFinder();
        //blockFinder.init(hardwareMap, true);

        waitForStart();

        int blockPosition = 1;//blockFinder.getBlockPosition();
        telemetry.addData("blockPosition", blockPosition);
        telemetry.update();

        drive.setAngle(0);

        if (blockPosition == 0 || blockPosition == 2) {
            drive.drive(29);
            drive.turn(90);
            if (blockPosition == 0) drive.drive(20);
            else if (blockPosition == 2) drive.drive(-3);
            assembly.setIntakePower(-1);
            sleep(500);
            drive.driveToDistance(6, 18);
            assembly.setIntakePower(0);
            drive.setAngle(90);
            drive.drive(24);
        } else if (blockPosition == 1) {
            drive.drive(48);
            assembly.setIntakePower(-1);
            sleep(500);
            drive.drive(6);
            assembly.setIntakePower(0);
        }
    }
}
