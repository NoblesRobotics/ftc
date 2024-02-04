package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.swerve.SwerveModule;

@TeleOp
public class TurnInPlaceStates extends LinearOpMode {
    @Override
    public void runOpMode() {
        SwerveModule[] modules = new SwerveModule[] {
                new SwerveModule(hardwareMap, 0, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 1, DcMotorSimple.Direction.FORWARD),
                new SwerveModule(hardwareMap, 2, DcMotorSimple.Direction.REVERSE),
                new SwerveModule(hardwareMap, 3, DcMotorSimple.Direction.REVERSE)
        };
        int[] servoToStateMap = new int[] {3, 1, 0, 2};

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

        DriveState driveState = DriveState.NOT_MOVING;

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

            boolean isAnyInput = (
                    Math.abs(gamepad1.left_stick_x) > 0.25 ||
                    Math.abs(gamepad1.left_stick_y) > 0.25 ||
                    Math.abs(gamepad1.right_stick_x) > 0.25
            );
            switch (driveState) {
                case NOT_MOVING:
                    for (SwerveModule module : modules) {
                        module.updateServo();
                        module.setPower(0);
                    }

                    if (isAnyInput) driveState = DriveState.TURNING;
                    break;

                case TURNING:
                    boolean anyServoMoving = false;
                    for (int i = 0; i < 4; i++) {
                        modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                        modules[i].updateServo();
                        modules[i].setPower(0);
                        anyServoMoving |= modules[i].isServoMoving();
                    }

                    if (!anyServoMoving) driveState = DriveState.DRIVING;
                    break;

                case DRIVING:
                    for (int i = 0; i < 4; i++) {
                        modules[i].setAngle(states[servoToStateMap[i]].angle.getDegrees());
                        modules[i].updateServo();
                        modules[i].setPower(states[servoToStateMap[i]].speedMetersPerSecond / maxSpeed); // DOES MAPPING APPLY???
                    }

                    if (!isAnyInput) driveState = DriveState.NOT_MOVING;
                    break;
            }
        }
    }

    enum DriveState {
        NOT_MOVING,
        TURNING,
        DRIVING
    }
}
