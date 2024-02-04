package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.SinglePress;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveServoStorage;
import org.firstinspires.ftc.teamcode.nobles.swerve.TeleOpSwerveDrive;

@TeleOp
public class TurnInPlaceClearCache extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveServoStorage.hasCachedPositions = false;
        AttributeTelemetry.setTelemetry(telemetry);

        TeleOpSwerveDrive drive = new TeleOpSwerveDrive(hardwareMap);
        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        Servo droneLauncher = hardwareMap.get(Servo.class, "servo4");

        drive.calibrateServos();
        assembly.setFlaps(true);
        assembly.getSlide().setSafety(true);

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

        SinglePress slideSafetyButton = new SinglePress(() -> {
            assembly.getSlide().setSafety(!assembly.getSlide().safetyOn);
        });

        final double[] driveSign = {1};
        AttributeTelemetry.set("Drive Sign", String.valueOf(driveSign[0]));
        SinglePress driveSignButton = new SinglePress(() -> {
            if (driveSign[0] == -1) driveSign[0] = 1;
            else driveSign[0] = -1;
            AttributeTelemetry.set("Drive Sign", String.valueOf(driveSign[0]));
        });

        while (opModeIsActive()) {
            drive.update(
                    gamepad1.left_stick_y * driveSign[0],
                    gamepad1.left_stick_x * driveSign[0],
                    gamepad1.right_stick_x
            );

            driveSignButton.update(gamepad1.left_bumper);

            if (gamepad2.x) droneLauncher.setPosition(0);
            else droneLauncher.setPosition(0.35);

            intakeButton.update(gamepad2.right_trigger > 0); // updates intakePower[0]
            if (gamepad2.right_bumper) intakePower[0] = -1;
            else if (intakePower[0] == -1) intakePower[0] = 0;
            assembly.setIntakePower(intakePower[0]);

            flapButton.update(gamepad2.left_trigger > 0);

            raiseButton.update(gamepad2.a);
            lowerButton.update(gamepad2.b);

            slideSafetyButton.update(gamepad2.dpad_down);

            assembly.getActionQueuer().update();

            if (gamepad2.y) {
                drive.resetServos();
                break;
            }
        }
    }
}
