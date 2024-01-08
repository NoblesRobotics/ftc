package org.firstinspires.ftc.teamcode.nobles.slidetest;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SlideAssembly {
    private final DoubleSlide slide;
    private final Servo rightFlap, leftFlap, pivot;
    private final DcMotorEx intake;
    private final CRServo intakeRoller;

    public SlideAssembly(HardwareMap hardwareMap) {
        slide = new DoubleSlide(hardwareMap);
        rightFlap = hardwareMap.get(Servo.class, "servo6");
        leftFlap = hardwareMap.get(Servo.class, "servo7");
        pivot = hardwareMap.get(Servo.class, "servo8");
        intake = hardwareMap.get(DcMotorEx.class, "motor6");
        intakeRoller = hardwareMap.get(CRServo.class, "servo5");
    }

    public void openFlaps() {
        leftFlap.setPosition(0.9);
        rightFlap.setPosition(0.1);
    }

    public void closeFlaps() {
        leftFlap.setPosition(1);
        rightFlap.setPosition(0);
    }

    public void raiseAssembly() {
        slide.setPosition(200);
        pivot.setPosition(0.01);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        slide.setPosition(2000);
        pivot.setPosition(0.5);
    }

    public void lowerAssembly() {
        pivot.setPosition(0.01);
        try {
            sleep(1500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        slide.setPosition(200);
        pivot.setPosition(0.2);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        slide.setPosition(0);
    }

    public void setIntakePower(double power) {
        intakeRoller.setPower(power);
        intake.setPower(-power);
    }
}
