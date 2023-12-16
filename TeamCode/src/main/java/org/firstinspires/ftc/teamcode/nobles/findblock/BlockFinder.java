package org.firstinspires.ftc.teamcode.nobles.findblock;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BlockFinder {
    private OpenCvCamera camera;
    private BlockFinderPipeline pipeline;

    public void init(HardwareMap hardwareMap, boolean useRed) {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        pipeline = new BlockFinderPipeline(useRed);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public int getBlockPosition() {
        ElapsedTime timer = new ElapsedTime();
        while (pipeline.getBlockPosition() == -1 && timer.seconds() < 3) Thread.yield();

        //camera.stopStreaming();
        return Math.max(pipeline.getBlockPosition(), 0);
    }
}
