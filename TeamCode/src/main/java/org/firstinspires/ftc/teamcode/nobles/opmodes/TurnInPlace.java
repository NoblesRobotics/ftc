package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nobles.SinglePress;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.TeleOpSwerveDrive;

@TeleOp
public class TurnInPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;

        TeleOpSwerveDrive drive = new TeleOpSwerveDrive(hardwareMap);
        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        Servo droneLauncher = hardwareMap.get(Servo.class, "servo4");

        drive.calibrateServos();
        waitForStart();

        final double[] intakePower = {0};
        SinglePress intakeButton = new SinglePress(() -> {
            if (intakePower[0] == 1) intakePower[0] = 0;
            else intakePower[0] = 1;
        });
        SinglePress flapButton = new SinglePress(() -> {
            assembly.setFlaps(!assembly.flapsOpen);
        });
        SinglePress raiseButton = new SinglePress(assembly::raiseAssembly);
        SinglePress lowerButton = new SinglePress(assembly::lowerAssembly);

        while (opModeIsActive()) {
            drive.update(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            if (gamepad2.x) droneLauncher.setPosition(0);
            else droneLauncher.setPosition(0.35);

            raiseButton.update(gamepad2.a);
            lowerButton.update(gamepad2.b);

            intakeButton.update(gamepad2.right_trigger > 0); // updates intakePower[0]
            if (gamepad2.right_bumper) intakePower[0] = -1;
            else if (intakePower[0] == -1) intakePower[0] = 0;
            assembly.setIntakePower(intakePower[0]);

            flapButton.update(gamepad2.left_trigger > 0);

            assembly.updateActionQueuer();

            if (gamepad2.y) {
                drive.resetServos();
                break;
            }
        }
    }
}
