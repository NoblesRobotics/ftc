package org.firstinspires.ftc.teamcode.nobles.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RatioMeasure extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveServo servo = new SwerveServo(hardwareMap, 0);

        while (!isStarted()) servo.calibrate();
        waitForStart();

        /*double target = 0;
        while (opModeIsActive()) {
            servo.setAngle(target);
            servo.update();
            if (gamepad1.dpad_left) target -= 0.1;
            else if (gamepad1.dpad_right) target += 0.1;
            telemetry.addData("target", target);
            telemetry.update();
        }*/

        servo.setAngle(180);
        while (servo.isMoving()) servo.update();
        sleep(1000);
        servo.setAngle(360);
        while (servo.isMoving()) servo.update();
        sleep(1000);
    }
}
