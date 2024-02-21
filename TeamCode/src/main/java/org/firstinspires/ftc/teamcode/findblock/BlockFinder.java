package org.firstinspires.ftc.teamcode.findblock;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class BlockFinder {
    private final OpenCvCamera camera;
    private final BlockFinderPipeline pipeline;

    public BlockFinder(HardwareMap hardwareMap, boolean useRed) {
        // Boilerplate OpenCV setup
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // processFrame in the pipeline will be called with every new frame
        pipeline = new BlockFinderPipeline(useRed);
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(544, 288, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    public int getBlockPosition() {
        pipeline.makeCapture();
        // Wait until capture is complete
        while (pipeline.isCapturing) Thread.yield();
        return pipeline.blockPosition;
    }
}
