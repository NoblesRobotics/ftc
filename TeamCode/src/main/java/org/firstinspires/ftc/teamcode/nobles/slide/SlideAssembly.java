package org.firstinspires.ftc.teamcode.nobles.slide;

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

    private final ActionQueuer actionQueuer = new ActionQueuer();

    public SlideAssembly(HardwareMap hardwareMap) {
        slide = new DoubleSlide(hardwareMap);
        rightFlap = hardwareMap.get(Servo.class, "servo6");
        leftFlap = hardwareMap.get(Servo.class, "servo7");
        pivot = hardwareMap.get(Servo.class, "servo8");
        intake = hardwareMap.get(DcMotorEx.class, "motor6");
        intakeRoller = hardwareMap.get(CRServo.class, "servo5");
    }

    public boolean flapsOpen = true;

    public void setFlaps(boolean setToOpen) {
        if (setToOpen) {
            actionQueuer.add(
                    new ActionQueuer.ServoAction(leftFlap, 0.8, 0),
                    new ActionQueuer.ServoAction(rightFlap, 0.385, 500)
            );
        } else {
            actionQueuer.add(
                    new ActionQueuer.ServoAction(leftFlap, 0.84, 0),
                    new ActionQueuer.ServoAction(rightFlap, 0.345, 500)
            );
        }
        flapsOpen = setToOpen;
    }

    public void raiseAssembly() {
        if (flapsOpen) setFlaps(false);
        actionQueuer.add(
                new ActionQueuer.SlideAction(slide, 2000),
                new ActionQueuer.ServoAction(pivot, 0.686, 500)
        );
    }

    public void lowerAssembly() {
        if (flapsOpen) setFlaps(false);
        actionQueuer.add(
                new ActionQueuer.ServoAction(pivot, 0.453, 500),
                new ActionQueuer.SlideAction(slide, 0)
        );
        if (!flapsOpen) setFlaps(true);
    }

    public void updateActionQueuer() {
        actionQueuer.update();
    }

    public void setIntakePower(double power) {
        intakeRoller.setPower(power);
        intake.setPower(-power);
    }
}
