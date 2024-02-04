package org.firstinspires.ftc.teamcode.nobles.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;

public class DoubleSlide {
    private final int MAX_DIFFERENCE = 200;
    private final double SLIDE_POWER = 0.6;

    private final SlideAssembly assembly;
    private final DcMotorEx slideA, slideB;

    public DoubleSlide(HardwareMap hardwareMap, SlideAssembly assembly) {
        this.assembly = assembly;

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

        slideA.setPower(SLIDE_POWER);
        slideB.setPower(SLIDE_POWER);
    }

    public void setPosition(int position) {
        slideA.setPower(SLIDE_POWER);
        slideB.setPower(SLIDE_POWER);

        slideA.setTargetPosition(position);
        slideB.setTargetPosition(position);
    }

    public boolean safetyOn = true;

    public void setSafety(boolean safetyOn) {
        this.safetyOn = safetyOn;
        AttributeTelemetry.set("Slide Safety", safetyOn ? "On" : "Off");
    }

    public boolean isBusy() {
        if (Math.abs(slideA.getCurrentPosition() - slideB.getCurrentPosition()) > MAX_DIFFERENCE) {
            slideA.setPower(0);
            slideB.setPower(0);
            assembly.getActionQueuer().clear();
            AttributeTelemetry.set("Slide Safety", "Triggered");
        }

        return slideA.isBusy() || slideB.isBusy();
    }
}
