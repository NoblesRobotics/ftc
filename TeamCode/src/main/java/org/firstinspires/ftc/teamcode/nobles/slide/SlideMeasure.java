package org.firstinspires.ftc.teamcode.nobles.slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlideMeasure extends LinearOpMode {
    @Override
    public void runOpMode() {
        DoubleSlide slide = new DoubleSlide(hardwareMap);
        waitForStart();
        int target = 0;
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) target += 1;
            else if (gamepad1.dpad_down) target -= 1;
            slide.setPosition(target);
            //telemetry.addData("actual", slide.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
