package org.firstinspires.ftc.teamcode.nobles.findblock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CameraTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        BlockFinder blockFinder = new BlockFinder();
        blockFinder.init(hardwareMap, true);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("position", blockFinder.getBlockPosition());
            telemetry.update();
        }
    }
}
