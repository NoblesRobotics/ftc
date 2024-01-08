package org.firstinspires.ftc.teamcode.nobles.slidetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SlideTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DoubleSlide slide = new DoubleSlide(hardwareMap);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "motor6");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "servo5");
        Servo rightFlap = hardwareMap.get(Servo.class, "servo6");
        Servo leftFlap = hardwareMap.get(Servo.class, "servo7");
        Servo pivot = hardwareMap.get(Servo.class, "servo8");
        pivot.setPosition(0.15);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                slide.setPosition(200);
                pivot.setPosition(0.01);
                sleep(2000);
                slide.setPosition(2000);
                pivot.setPosition(0.5);
            } else if (gamepad1.b) {
                pivot.setPosition(0.01);
                sleep(1500);
                slide.setPosition(200);
                pivot.setPosition(0.15);
                sleep(1000);
                slide.setPosition(0);
            }

            if (gamepad1.left_bumper) {
                intake.setPower(-1);
                intakeRoller.setPower(1);
            } else {
                intake.setPower(0);
                intakeRoller.setPower(0);
            }

            if (gamepad1.left_bumper) {
                leftFlap.setPosition(0.9);
                rightFlap.setPosition(0.1);
            } else if (gamepad1.right_bumper) {
                leftFlap.setPosition(1);
                rightFlap.setPosition(0);
            }

            /*if (gamepad1.a) {
                slide.setPosition(200);
                pivot.setPosition(0.01);
            } else if (gamepad1.x) {
                pivot.setPosition(0.15);
                sleep(1000);
                slide.setPosition(0);
            } else if (gamepad1.y) {
                slide.setPosition(1000);
                pivot.setPosition(0.5);
            } else if (gamepad1.b) {
                pivot.setPosition(0.01);
                sleep(3000);
                slide.setPosition(200);
            }*/
        }
    }
}
