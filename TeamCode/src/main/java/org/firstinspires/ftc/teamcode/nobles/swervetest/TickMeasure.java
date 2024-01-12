package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TickMeasure extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor0");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) motor.setPower(0.1);
            else if (gamepad1.dpad_down) motor.setPower(-0.1);
            else motor.setPower(0);
            telemetry.addData("ticks", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
