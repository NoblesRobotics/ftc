package org.firstinspires.ftc.teamcode.nobles.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.nobles.AttributeTelemetry;
import org.firstinspires.ftc.teamcode.nobles.findblock.BlockFinder;
import org.firstinspires.ftc.teamcode.nobles.slide.SlideAssembly;
import org.firstinspires.ftc.teamcode.nobles.swerve.AutoSwerveDrive;

public class SwerveAuto {
    private final boolean isBackdropOnLeft, shouldPark, shouldScore;
    private final BlockFinder blockFinder;
    private final AutoSwerveDrive drive;
    private final SlideAssembly assembly;

    public SwerveAuto(HardwareMap hardwareMap, boolean isBackdropOnLeft, boolean shouldPark, boolean shouldScore) {
        this.isBackdropOnLeft = isBackdropOnLeft;
        this.shouldPark = shouldPark;
        this.shouldScore = shouldScore;

        blockFinder = new BlockFinder(hardwareMap, !isBackdropOnLeft);

        drive = new AutoSwerveDrive(hardwareMap);
        drive.calibrateServos();

        assembly = new SlideAssembly(hardwareMap);
        assembly.setFlaps(false);
        assembly.getActionQueuer().idleOnBusy();
    }

    public void start() {
        ElapsedTime overallTimer = new ElapsedTime();
        overallTimer.reset();

        boolean isBlockInCenter = blockFinder.isBlockDetected();
        AttributeTelemetry.set("Is Block In Center", String.valueOf(isBlockInCenter));

        if (isBlockInCenter) {
            // Turn around
            drive.drive(6);
            drive.turn(180);

            // Drive to line
            drive.drive(-22);

            // Spit out pixel while driving forward
            assembly.setIntakePower(-0.5);
            sleep(500);
            drive.drive(4);
            assembly.setIntakePower(0);

            // Drive back
            drive.drive(18);
        } else {
            // Turn to see left position
            drive.drive(6);
            drive.turn(isBackdropOnLeft ? 30 : -30);

            boolean isBlockOnLeft = blockFinder.isBlockDetected();
            AttributeTelemetry.set("Is Block On Left", String.valueOf(isBlockOnLeft));

            // Drive to middle of square and face backdrop
            drive.turn(0);
            drive.drive(22);
            drive.turn(((isBackdropOnLeft && !isBlockOnLeft) || (!isBackdropOnLeft && isBlockOnLeft)) ? 90 : -90);

            // Drive to line
            drive.drive(-3);

            // Spit out pixel while driving forward
            assembly.setIntakePower(-0.5);
            sleep(500);
            drive.drive(4);
            assembly.setIntakePower(0);

            // Strafe out of way
            /*drive.setAngle(((isBackdropOnLeft && !isBlockOnLeft) || (!isBackdropOnLeft && isBlockOnLeft)) ? 90 : -90);
            drive.drive(24, 1);*/
            drive.setAngle(0);
        }

        drive.resetServos();
    }

    private void score() {
        assembly.raiseAssembly(1650);
        assembly.getActionQueuer().idleOnBusy();

        assembly.setFlaps(true);
        assembly.getActionQueuer().idleOnBusy();

        assembly.lowerAssembly(25);
        assembly.getActionQueuer().idleOnBusy();
    }

    private void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
