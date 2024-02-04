package org.firstinspires.ftc.teamcode.nobles.findblock;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlockFinderPipeline extends OpenCvPipeline {
    private final boolean useRed;

    public BlockFinderPipeline(boolean useRed) {
        this.useRed = useRed;
    }

    public boolean isCapturing = false, blockDetected = false;

    public void makeCapture() {
        isCapturing = true;
    }

    @Override
    public Mat processFrame(Mat frame) {
        if (isCapturing) {
            blockDetected = isBlockDetected(frame);
            isCapturing = false;
        }

        return frame;
    }

    private boolean isBlockDetected(Mat frame) {
        int targetChannel = useRed ? 0 : 2;
        int removedChannel = useRed ? 2 : 0;

        int sideRestriction = frame.width() / 5;
        double[][] filteredData = new double[frame.height()][frame.width() - 2 * sideRestriction];
        for (int y = 0; y < frame.height(); y++) {
            for (int x = sideRestriction; x < frame.width() - sideRestriction; x++) {
                double[] pixel = frame.get(y, x);
                filteredData[y][x - sideRestriction] = pixel[targetChannel] - (pixel[removedChannel] + pixel[1]) / 2;
            }
        }
        int filteredDataSize = filteredData.length * filteredData[0].length;

        int colorThreshold = 250, lastCountAboveThreshold = 0;
        while (colorThreshold > 50) {
            int countAboveThreshold = 0;
            for (double[] row : filteredData) {
                for (double pixel : row) {
                    if (pixel > colorThreshold) countAboveThreshold++;
                }
            }

            if (countAboveThreshold > 0.05 * filteredDataSize && countAboveThreshold > 1.5 * lastCountAboveThreshold) {
                return true;
            }

            colorThreshold -= 5;
        }

        return false;
    }
}
