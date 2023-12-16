package org.firstinspires.ftc.teamcode.nobles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class GoodAuto extends LinearOpMode {
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
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            for (DcMotorEx motor : motors) motor.setPower(0.25);
            for (Servo servo : servos) servo.setPosition(0.5);
        }
    }
}
