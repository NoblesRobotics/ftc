package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.AutoSwerveDrive;

@Autonomous
public class RedFar extends LinearOpMode {
    private SlideAssembly assembly;

    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 108, 8.75, -1);
        drive.calibrateServos();
        assembly = new SlideAssembly(hardwareMap);
        assembly.setFlaps(false);
        assembly.getActionQueuer().idleOnBusy();
        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        waitForStart();

        int blockPosition = 2 - blockFinder.getBlockPosition();
        AttributeTelemetry.set("Block Position", String.valueOf(blockPosition));

        drive.setAngle(-90);
        drive.drive(6);
        drive.turn(180);
        drive.snapPosition(108, 14.75);

        if (blockPosition == 0) {
            drive.driveToPosition(108, 36);
            drive.driveToPosition(104.75 - 17, 36);
            dumpToScore();
        } else if (blockPosition == 1) {
            drive.driveToPosition(108, 48);
            dumpToScore();
        } else if (blockPosition == 2) {
            drive.driveToPosition(110.25, 36);
            dumpToScore();
        }
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
