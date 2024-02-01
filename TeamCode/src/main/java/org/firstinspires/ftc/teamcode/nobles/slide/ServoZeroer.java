package org.firstinspires.ftc.teamcode.nobles.slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoZeroer extends LinearOpMode {
    @Override
    public void runOpMode() {
        // left flap - open .8, closed .84
        // right flap - open .385, closed .345
        // pivot - stowed .453, board .686

        /*DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "motor6");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "servo5");

        Servo rightFlap = hardwareMap.get(Servo.class, "servo6");
        Servo leftFlap = hardwareMap.get(Servo.class, "servo7");
        Servo pivot = hardwareMap.get(Servo.class, "servo8");*/

        Servo drone = hardwareMap.get(Servo.class, "servo4");

        waitForStart();
        double target = drone.getPosition();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) target += 0.001;
            else if (gamepad1.dpad_down) target -= 0.001;
            drone.setPosition(target);
            /*if (gamepad1.a) {
                leftFlap.setPosition(0.8);
                rightFlap.setPosition(0.385);
            } else {
                leftFlap.setPosition(0.84);
                rightFlap.setPosition(0.345);
            }
            if (gamepad1.x) {
                intakeRoller.setPower(1);
                intake.setPower(-1);
            } else {
                intakeRoller.setPower(0);
                intake.setPower(0);
            }
            if (gamepad1.y) {
                pivot.setPosition(.686);
            } else {
                pivot.setPosition(.453);
            }*/
            telemetry.addData("actual", drone.getPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
