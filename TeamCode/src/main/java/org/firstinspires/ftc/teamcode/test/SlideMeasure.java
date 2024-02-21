package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.slide.DoubleSlide;
import org.firstinspires.ftc.teamcode.swerve.servo.SwerveServo;

@TeleOp
public class SlideMeasure extends LinearOpMode {
    @Override
    public void runOpMode() {
        DoubleSlide slide = new DoubleSlide(hardwareMap, null);
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

    @TeleOp
    public static class ServoRatioMeasure extends LinearOpMode {
        @Override
        public void runOpMode() {
            // This class was used to tune REV_RATIO in SwerveServo
            // With REV_RATIO set to 1, I ran this class and visually rotated the servo to 180 degrees
            // and found the robot to report the servo angle as 221 degrees
            // So I set REV_RATIO to 221 / 180

            SwerveServo servo = new SwerveServo(hardwareMap, 0);

            while (!isStarted()) servo.calibrate();
            waitForStart();

            double target = 0;
            while (opModeIsActive()) {
                servo.setAngle(target);
                servo.update();
                if (gamepad1.dpad_left) target -= 0.1;
                else if (gamepad1.dpad_right) target += 0.1;
                telemetry.addData("target", target);
                telemetry.update();
            }
        }
    }
}
