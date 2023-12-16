package org.firstinspires.ftc.teamcode.nobles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx[] motors = new DcMotorEx[] {
                hardwareMap.get(DcMotorEx.class, "motor0"),
                hardwareMap.get(DcMotorEx.class, "motor1"),
                hardwareMap.get(DcMotorEx.class, "motor2"),
                hardwareMap.get(DcMotorEx.class, "motor3")
        };
        Servo[] servos = new Servo[] {
                hardwareMap.get(Servo.class, "servo0"),
                hardwareMap.get(Servo.class, "servo1"),
                hardwareMap.get(Servo.class, "servo2"),
                hardwareMap.get(Servo.class, "servo3")
        };
        //DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        double targetPosition = 0.5;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.right_stick_x > 0) {
                motors[0].setPower(-1);
                motors[1].setPower(-1);
                motors[2].setPower(1);
                motors[3].setPower(1);
            } else if (gamepad1.right_stick_x < 0) {
                motors[0].setPower(1);
                motors[1].setPower(1);
                motors[2].setPower(-1);
                motors[3].setPower(-1);
            } else {
                for (DcMotorEx motor : motors) motor.setPower(Math.signum(gamepad1.left_stick_y) * Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)));
            }
            for (Servo servo : servos) servo.setPosition(targetPosition);
            /*if (gamepad1.left_trigger > 0) intakeMotor.setPower(1);
            else if (gamepad1.right_trigger > 0) intakeMotor.setPower(-1);
            else intakeMotor.setPower(0);*/

            double deltaMove = (gamepad1.left_stick_x * 0.5 + 0.5) - targetPosition;
            targetPosition += (Math.abs(deltaMove) > 0.05 ? Math.signum(deltaMove) : 0) * timer.seconds() * 3;
            timer.reset();
        }
    }
}
