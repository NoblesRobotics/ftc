package org.firstinspires.ftc.teamcode.slide;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AttributeTelemetry;

public class SlideAssembly {
    private final DoubleSlide slide;
    private final Servo rightFlap, leftFlap, pivot;
    private final DcMotorEx intake;
    private final CRServo intakeRoller;

    private final ActionQueuer actionQueuer = new ActionQueuer();

    public SlideAssembly(HardwareMap hardwareMap) {
        slide = new DoubleSlide(hardwareMap, this);
        rightFlap = hardwareMap.get(Servo.class, "servo6");
        leftFlap = hardwareMap.get(Servo.class, "servo7");
        pivot = hardwareMap.get(Servo.class, "servo8");
        pivot.setPosition(0.33);
        intake = hardwareMap.get(DcMotorEx.class, "motor6");
        intakeRoller = hardwareMap.get(CRServo.class, "servo5");
    }

    public boolean flapsOpen = true;

    public void setFlaps(boolean setToOpen) {
        // Uses measured setpoints to open and close flaps that hold pixels
        if (setToOpen) {
            actionQueuer.add(
                    new ActionQueuer.ServoAction(leftFlap, 0.653, 0),
                    new ActionQueuer.ServoAction(rightFlap, 0.147, 500)
            );
        } else {
            actionQueuer.add(
                    new ActionQueuer.ServoAction(leftFlap, 0.502, 0),
                    new ActionQueuer.ServoAction(rightFlap, 0, 500)
            );
        }
        flapsOpen = setToOpen;
        AttributeTelemetry.set("Flaps", flapsOpen ? "Open" : "Closed");
    }

    public void raiseAssembly() {
        raiseAssembly(2000);
    }

    public void raiseAssembly(int slidePosition) {
        if (flapsOpen) setFlaps(false);
        actionQueuer.add(
                new ActionQueuer.SlideAction(slide, slidePosition),
                // Swing out pivot to setpoint
                new ActionQueuer.ServoAction(pivot, 0.6, 500)
        );
    }

    public void lowerAssembly() {
        lowerAssembly(0);
    }

    public void lowerAssembly(int slidePosition) {
        if (flapsOpen) setFlaps(false);
        actionQueuer.add(
                // Pull pivot back in to setpoint
                new ActionQueuer.ServoAction(pivot, 0.33, 500),
                new ActionQueuer.SlideAction(slide, slidePosition)
        );
        if (!flapsOpen) setFlaps(true);
    }

    public void setIntakePower(double power) {
        intakeRoller.setPower(power); // Belts inside robot
        intake.setPower(-power); // Actual pixel grabber
    }

    public ActionQueuer getActionQueuer() {
        return actionQueuer;
    }

    public DoubleSlide getSlide() {
        return slide;
    }
}
