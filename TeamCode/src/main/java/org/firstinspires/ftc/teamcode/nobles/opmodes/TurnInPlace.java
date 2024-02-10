package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.SinglePress;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.TeleOpSwerveDrive;

@TeleOp
public class TurnInPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        TeleOpSwerveDrive drive = new TeleOpSwerveDrive(hardwareMap);
        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        Servo droneLauncher = hardwareMap.get(Servo.class, "servo4");

        drive.calibrateServos();
        assembly.setFlaps(true);
        assembly.getSlide().setSafety(true);

        DcMotorEx climber = hardwareMap.get(DcMotorEx.class, "motor7");
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climber.setTargetPosition(0);
        climber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        climber.setPower(1);
        int climberPosition = 1700;

        waitForStart();

        final double[] intakePower = {0};
        SinglePress intakeButton = new SinglePress(() -> {
            if (intakePower[0] == 1) intakePower[0] = 0;
            else intakePower[0] = 1;
        });

        SinglePress flapButton = new SinglePress(() -> {
            assembly.setFlaps(!assembly.flapsOpen);
        });

        SinglePress raiseHighButton = new SinglePress(() -> assembly.raiseAssembly(2000));
        SinglePress raiseLowButton = new SinglePress(() -> assembly.raiseAssembly(1400));
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

        ElapsedTime dropTimer = new ElapsedTime();

        while (opModeIsActive()) {
            drive.update(
                    gamepad1.left_stick_y * driveSign[0],
                    gamepad1.left_stick_x * driveSign[0],
                    gamepad1.right_stick_x
            );

            driveSignButton.update(gamepad1.left_bumper);

            if (gamepad2.y) droneLauncher.setPosition(0);
            else droneLauncher.setPosition(0.35);

            intakeButton.update(gamepad2.right_trigger > 0); // updates intakePower[0]
            if (gamepad2.right_bumper) intakePower[0] = -1;
            else if (intakePower[0] == -1) intakePower[0] = 0;
            assembly.setIntakePower(intakePower[0]);

            flapButton.update(gamepad2.left_trigger > 0);

            raiseHighButton.update(gamepad2.x);
            raiseLowButton.update(gamepad2.a);
            lowerButton.update(gamepad2.b);

            slideSafetyButton.update(gamepad2.dpad_right);

            assembly.getActionQueuer().update();

            climber.setTargetPosition(climberPosition);
            if (dropTimer.seconds() > 3) {
                if (gamepad2.dpad_up) climberPosition += 50;
                else if (gamepad2.dpad_down) climberPosition -= 50;
                else climberPosition = climber.getCurrentPosition();
            }
            AttributeTelemetry.set("Climber Position", String.valueOf(climber.getCurrentPosition()));
        }
    }
}
