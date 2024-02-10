package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.AutoSwerveDrive;

@TeleOp
public class FarTest extends LinearOpMode {
    private SlideAssembly assembly;

    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 108, 8.75, 1);
        drive.calibrateServos();
        assembly = new SlideAssembly(hardwareMap);

        waitForStart();

        int blockPosition = 2;

        if (blockPosition == 0) {
            drive.driveToPosition(108, 36);
            drive.turn(-90);
            drive.driveToPosition(104.75, 36);
            dumpToScore();
            drive.driveToPosition(108,36);
            drive.turn(90);
        } else if (blockPosition == 1) {
            drive.driveToPosition(106, 48);
            dumpToScore();
        } else if (blockPosition == 2) {
            drive.driveToPosition(111.25, 36);
            dumpToScore();
        }
        drive.driveToPosition(108, 60);
        drive.driveToPosition(24, 60);
        drive.driveToPosition(22.5, 36);
        drive.turn(90);
        drive.driveToDistance(2);
        placeToScore();
        drive.driveToPosition(22.5, 12);
        drive.resetServos();
    }

    private void dumpToScore() {
        assembly.setIntakePower(-0.1);
        sleep(1000);
        assembly.setIntakePower(0);
    }

    private void placeToScore() {
        assembly.raiseAssembly(1400);
        assembly.setFlaps(true);
        assembly.lowerAssembly();
    }
}
