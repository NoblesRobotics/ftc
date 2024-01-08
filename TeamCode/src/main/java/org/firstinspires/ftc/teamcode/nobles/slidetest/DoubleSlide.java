package org.firstinspires.ftc.teamcode.nobles.slidetest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleSlide {
    private final int MAX_DIFFERENCE = 100;
    private final DcMotorEx slideA, slideB;

    public DoubleSlide(HardwareMap hardwareMap) {
        slideA = hardwareMap.get(DcMotorEx.class, "motor4");
        slideB = hardwareMap.get(DcMotorEx.class, "motor5");

        slideA.setDirection(DcMotorSimple.Direction.REVERSE);
        slideB.setDirection(DcMotorSimple.Direction.FORWARD);

        slideA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideA.setTargetPosition(0);
        slideB.setTargetPosition(0);

        slideA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideA.setPower(0.6);
        slideB.setPower(0.6);
    }

    public void setPosition(int position) {
        slideA.setTargetPosition(position);
        slideB.setTargetPosition(position);

        while (slideA.isBusy() || slideB.isBusy()) {
            if (Math.abs(slideA.getCurrentPosition() - slideB.getCurrentPosition()) > MAX_DIFFERENCE) {
                slideA.setPower(0);
                slideB.setPower(0);
                while (true) Thread.yield();
            }
        }
    }
}
