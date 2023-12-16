package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OneServoSpin extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveServo servo = new SwerveServo(hardwareMap, 0);

        while (!isStarted()) {
            servo.calibrate();
        }
        waitForStart();

        while (opModeIsActive()) {
            double target = Math.toDegrees(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x));
            servo.setAngle(target);
            servo.update();
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}

class SwervePositionWrapper {
    private final SwerveServo servo;

    public SwervePositionWrapper(SwerveServo servo) {
        this.servo = servo;
    }

    public void setAngle(double angle) {

    }
}