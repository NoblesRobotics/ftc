package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.swerve.AutoSwerveDrive;

@Autonomous
public class BlueFar extends LinearOpMode {
    private SlideAssembly assembly;

    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 108, 8.75, 1);
        drive.calibrateServos();
        assembly = new SlideAssembly(hardwareMap);
        assembly.setFlaps(false);
        assembly.getActionQueuer().idleOnBusy();
        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        waitForStart();

        int blockPosition = blockFinder.getBlockPosition();
        AttributeTelemetry.set("Block Position", String.valueOf(blockPosition));

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
