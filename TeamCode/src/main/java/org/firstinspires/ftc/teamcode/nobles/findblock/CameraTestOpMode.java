package org.firstinspires.ftc.teamcode.nobles.findblock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class CameraTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        waitForStart();

        while (opModeIsActive()) {
            AttributeTelemetry.set("Block Detected", String.valueOf(blockFinder.isBlockDetected()));
        }
    }
}