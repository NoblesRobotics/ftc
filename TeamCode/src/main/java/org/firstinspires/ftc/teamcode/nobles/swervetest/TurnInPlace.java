package org.firstinspires.ftc.teamcode.nobles.swervetest;

import com.acmerobotics.roadrunner.kinematics.SwerveKinematics;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.nobles.TelemetryStatic;
import org.firstinspires.ftc.teamcode.nobles.slidetest.DoubleSlide;
import org.firstinspires.ftc.teamcode.nobles.slidetest.SlideAssembly;

@TeleOp
public class TurnInPlace extends LinearOpMode {
    @Override
    public void runOpMode() {
        TelemetryStatic.telemetry = telemetry;

        SwerveModule[] modules = new SwerveModule[] {
                new SwerveModule(hardwareMap, 0, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 1, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 2, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 3, DcMotorSimple.Direction.REVERSE)
        };
        int[] servoToStateMap = new int[] {3, 1, 0, 2};

        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        Servo droneLauncher = hardwareMap.get(Servo.class, "servo4");

        ElapsedTime calibrationTimer = new ElapsedTime();
        while (!isStarted() || calibrationTimer.seconds() < 3) {
            for (SwerveModule module : modules) module.calibrateServo();
        }
        waitForStart();

        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                new Translation2d(0.203, 0.203),
                new Translation2d(0.203, -0.203),
                new Translation2d(-0.203, 0.203),
                new Translation2d(-0.203, -0.203)
        );

        double intakePower = 0;
        boolean intakeButtonPressed = false;
        boolean flapsOpen = true;
        boolean flapButtonPressed = false;

        while (opModeIsActive()) {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    )
            );

            double maxSpeed = 0;
            for (SwerveModuleState state : states) {
                if (Math.abs(state.speedMetersPerSecond) > maxSpeed) maxSpeed = Math.abs(state.speedMetersPerSecond);
            }

            for (int i = 0; i < 4; i++) {
                modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                modules[i].updateServo();
                modules[i].setPower(states[i].speedMetersPerSecond / maxSpeed);
            }

            if (gamepad1.x) droneLauncher.setPosition(1);
            else droneLauncher.setPosition(0.1);

            if (gamepad1.a) assembly.raiseAssembly();
            else if (gamepad1.b) assembly.lowerAssembly();

            if (gamepad1.left_bumper) assembly.openFlaps();
            else if (gamepad1.right_bumper) assembly.closeFlaps();

            if (gamepad1.right_trigger > 0 && !intakeButtonPressed) {
                if (intakePower == 1) intakePower = 0;
                else intakePower = 1;
                intakeButtonPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                intakeButtonPressed = false;
            }
            if (gamepad1.right_bumper) intakePower = -1;
            else if (intakePower == -1) intakePower = 0;
            assembly.setIntakePower(intakePower);

            if (gamepad1.left_trigger > 0 && !flapButtonPressed) {
                flapsOpen = !flapsOpen;
                if (flapsOpen) assembly.openFlaps();
                else assembly.closeFlaps();
                flapButtonPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                flapButtonPressed = false;
            }
        }
    }
}
