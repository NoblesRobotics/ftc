package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.findblock.BlockFinder;

@TeleOp
public class CameraTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        waitForStart();

        while (opModeIsActive()) {
            AttributeTelemetry.set("Block Detected", String.valueOf(blockFinder.getBlockPosition()));
        }
    }
}