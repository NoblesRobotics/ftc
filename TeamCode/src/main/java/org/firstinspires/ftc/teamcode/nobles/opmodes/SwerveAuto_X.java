package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.AutoSwerveDrive;

@TeleOp
public class SwerveAuto_X extends LinearOpMode {
    @Override
    public void runOpMode() {
        AttributeTelemetry.setTelemetry(telemetry);

        boolean backdropIsOnLeft = true;
        boolean shouldPark = true;

        BlockFinder blockFinder = new BlockFinder(hardwareMap, false);

        AutoSwerveDrive drive = new AutoSwerveDrive(hardwareMap);
        drive.calibrateServos();

        SlideAssembly assembly = new SlideAssembly(hardwareMap);
        assembly.setFlaps(false);
        assembly.getActionQueuer().idleOnBusy();

        waitForStart();

        boolean isBlockInCenter = blockFinder.isBlockDetected();
        AttributeTelemetry.set("Is Block In Center", String.valueOf(isBlockInCenter));

        if (isBlockInCenter) {
            // Drive to line
            drive.drive(47);

            // Spit out pixel while driving forward
            assembly.setIntakePower(-0.5);
            sleep(500);
            drive.drive(4);
            assembly.setIntakePower(0);

            if (shouldPark) {
                // Face backdrop and drive forward
                drive.turn(backdropIsOnLeft ? 90 : -90);
                drive.drive(12);

                // Strafe to align with backdrop
                drive.setAngle(backdropIsOnLeft ? 90 : -90);
                drive.drive(12);

                // Drive to backdrop
                drive.setAngle(0);
                drive.turn(backdropIsOnLeft ? 90 : -90);
                drive.driveToDistance(2);

                // Score
                score(assembly);
            }
        } else {
            // Turn to see left position
            drive.drive(6);
            drive.turn(backdropIsOnLeft ? 30 : -30);

            boolean isBlockOnLeft = blockFinder.isBlockDetected();
            AttributeTelemetry.set("Is Block On Left", String.valueOf(isBlockOnLeft));

            // Drive to middle of square and face backdrop
            drive.turn(0);
            drive.drive(22);
            drive.turn(backdropIsOnLeft ? 90 : -90);

            // Drive to line
            drive.drive(isBlockOnLeft ? 18 : -2); // CHANGED I CHANGED IT

            // Spit out pixel while driving forward
            assembly.setIntakePower(-0.5);
            sleep(500);
            drive.drive(4);
            assembly.setIntakePower(0);

            if (shouldPark) {
                if (isBlockOnLeft) {
                    // Strafe out of way of block
                    drive.setAngle(backdropIsOnLeft ? 90 : -90);
                    drive.drive(15, 0.8);

                    // Drive through block position
                    drive.setAngle(0);
                    drive.drive(6, 0.8);

                    // Strafe back to backdrop
                    drive.setAngle(backdropIsOnLeft ? -90 : 90);
                    drive.drive(9, 0.8);

                    // Reset angle
                    drive.setAngle(0);
                }

                // Drive to backdrop
                drive.driveToDistance(2);

                // Score
                score(assembly);
            }
        }

        drive.resetServos();
    }

    private void score(SlideAssembly assembly) {
        assembly.raiseAssembly(1650);
        assembly.getActionQueuer().idleOnBusy();

        assembly.setFlaps(true);
        assembly.getActionQueuer().idleOnBusy();

        assembly.lowerAssembly(25);
        assembly.getActionQueuer().idleOnBusy();
    }
}
