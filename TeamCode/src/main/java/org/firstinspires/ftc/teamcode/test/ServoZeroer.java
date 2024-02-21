package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoZeroer extends LinearOpMode {
    @Override
    public void runOpMode() {
        // left flap - open .653, closed .502
        // right flap - open .147, closed 0
        // pivot - stowed .382, board .6

        /*DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "motor6");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "servo5");*/

        Servo rightFlap = hardwareMap.get(Servo.class, "servo6");
        Servo leftFlap = hardwareMap.get(Servo.class, "servo7");
        //Servo pivot = hardwareMap.get(Servo.class, "servo8");

        //Servo drone = hardwareMap.get(Servo.class, "servo4");

        waitForStart();
        //double targetPivot = 0.6;
        double targetRight = .127;//rightFlap.getPosition();
        double targetLeft = .573;//leftFlap.getPosition();
        while (opModeIsActive()) {
            /*if (gamepad1.dpad_up) targetPivot += 0.001;
            else if (gamepad1.dpad_down) targetPivot -= 0.001;
            pivot.setPosition(targetPivot);
            telemetry.addData("actualPivot", pivot.getPosition());
            telemetry.addData("targetPivot", targetPivot);
            telemetry.update();*/
            if (gamepad1.dpad_up) targetRight += 0.001;
            else if (gamepad1.dpad_down) targetRight -= 0.001;
            rightFlap.setPosition(targetRight);
            telemetry.addData("actualRight", rightFlap.getPosition());
            telemetry.addData("targetRight", targetRight);
            if (gamepad1.dpad_right) targetLeft += 0.001;
            else if (gamepad1.dpad_left) targetLeft -= 0.001;
            leftFlap.setPosition(targetLeft);
            telemetry.addData("actualLeft", leftFlap.getPosition());
            telemetry.addData("targetLeft", targetLeft);
            telemetry.update();
        }
    }
}
