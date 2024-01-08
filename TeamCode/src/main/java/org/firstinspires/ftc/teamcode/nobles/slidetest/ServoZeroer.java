package org.firstinspires.ftc.teamcode.nobles.slidetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoZeroer extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo rightFlag = hardwareMap.get(Servo.class, "servo6");
        Servo leftFlag = hardwareMap.get(Servo.class, "servo7");
        Servo pivot = hardwareMap.get(Servo.class, "servo8");
        waitForStart();
        double target = pivot.getPosition();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) target += 0.001;
            else if (gamepad1.dpad_down) target -= 0.001;
            pivot.setPosition(target);
            telemetry.addData("actual", pivot.getPosition());
            telemetry.addData("target", target);
            telemetry.update();
            /*if (gamepad1.a) {
                rightFlag.setPosition(0.1);
                leftFlag.setPosition(0.9);
            } else {
                rightFlag.setPosition(0);
                leftFlag.setPosition(1);
            }*/
        }
    }
}
