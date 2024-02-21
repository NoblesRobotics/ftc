package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.swerve.AutoSwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.servo.SwerveServoStorage;

@Autonomous
public class BlueClosePark extends LinearOpMode {
    private SlideAssembly assembly;

    @Override
    public void runOpMode() {
        SwerveServoStorage.hasCachedPositions = false;
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 60, 8.75, 1);
        drive.calibrateServos();
        assembly = new SlideAssembly(hardwareMap);
        assembly.setFlaps(false);
        assembly.getActionQueuer().idleOnBusy();
        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        waitForStart();

        //drive.setAngle(35.9);
        //drive.drive(35.8, 2, 0, 0.2);
        /*drive.driveAtAngle(-21, 29);
        drive.setAngle(0);
        assembly.setIntakePower(-0.2);
        drive.drive(6);
        assembly.setIntakePower(0);*/

        int blockPosition = blockFinder.getBlockPosition();
        AttributeTelemetry.set("Block Position", String.valueOf(blockPosition));

        if (blockPosition == 0) {
            drive.driveToPosition(38.25, 39.25);
            dumpToScore();
            drive.driveToPosition(22.5, 29.75);
        } else if (blockPosition == 1) {
            drive.driveToPosition(51.25, 48);
            dumpToScore();
            drive.driveToPosition(22.5, 38);
        } else if (blockPosition == 2) {
            drive.driveToPosition(60, 30);
            drive.driveToPosition(63.25, 38.25);
            dumpToScore();
            drive.driveToPosition(22.5, 43.25);
        }
        //drive.turn(0);
        //drive.setAngle(0);
        drive.driveToDistance(2);
        //placeToScore();
        drive.driveToPosition(25, 12);
        drive.resetServos();
    }

    private void dumpToScore() {
        assembly.setIntakePower(-0.2);
        sleep(1000);
        assembly.setIntakePower(0);
    }

    private void placeToScore() {
        assembly.raiseAssembly(1400);
        assembly.getActionQueuer().idleOnBusy();
        assembly.setFlaps(true);
        assembly.getActionQueuer().idleOnBusy();
        assembly.lowerAssembly();
        assembly.getActionQueuer().idleOnBusy();
    }
}
