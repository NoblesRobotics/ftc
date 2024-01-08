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
            telemetry.addData("ticks", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
