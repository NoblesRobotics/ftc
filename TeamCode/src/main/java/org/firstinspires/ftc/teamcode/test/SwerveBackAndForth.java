package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.swerve.AutoSwerveDrive;

@TeleOp
public class SwerveBackAndForth extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap, 60, 8.75, 1);
        drive.calibrateServos();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double kp = 2, ki = 0, kd = 0.2, dist = 24;
        while (opModeIsActive()) {
            //drive.drive(dist, kp, ki, kd);

            timer.reset();
            while (timer.seconds() < 3) {
                if (gamepad1.dpad_up) kp += 0.05;
                else if (gamepad1.dpad_down) kp -= 0.05;
                if (gamepad1.dpad_right) kd += 0.05;
                else if (gamepad1.dpad_left) kd -= 0.05;
                if (gamepad1.a) dist++;
                else if (gamepad1.b) dist--;
                AttributeTelemetry.set("kp", String.valueOf(kp));
                AttributeTelemetry.set("kd", String.valueOf(kd));
                AttributeTelemetry.set("dist", String.valueOf(dist));
                sleep(100);
            }

            //drive.drive(-dist, kp, ki, kd);

            timer.reset();
            while (timer.seconds() < 3) {
                if (gamepad1.dpad_up) kp += 0.05;
                else if (gamepad1.dpad_down) kp -= 0.05;
                if (gamepad1.dpad_right) kd += 0.05;
                else if (gamepad1.dpad_left) kd -= 0.05;
                if (gamepad1.a) dist++;
                else if (gamepad1.b) dist--;
                AttributeTelemetry.set("kp", String.valueOf(kp));
                AttributeTelemetry.set("kd", String.valueOf(kd));
                AttributeTelemetry.set("dist", String.valueOf(dist));
                sleep(100);
            }
        }
    }
}
