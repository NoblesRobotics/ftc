package org.firstinspires.ftc.teamcode.nobles.findblock;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlockFinderPipeline extends OpenCvPipeline {
    private final boolean useRed;

    public BlockFinderPipeline(boolean useRed) {
        this.useRed = useRed;
    }

    public boolean isCapturing = false;

    public int blockPosition = -1;

    public void makeCapture() {
        isCapturing = true;
    }

    @Override
    public Mat processFrame(Mat frame) {
        if (isCapturing) {
            blockPosition = getBlockPosition(frame);
            isCapturing = false;
        }

        return frame;
    }

    private int getBlockPosition(Mat frame) {
        int targetChannel = useRed ? 0 : 2;
        int removedChannel = useRed ? 2 : 0;

        double[][] filteredData = new double[frame.height()][frame.width()];
        for (int y = 0; y < frame.height(); y++) {
            for (int x = 0; x < frame.width(); x++) {
                double[] pixel = frame.get(y, x);
                filteredData[y][x] = pixel[targetChannel] - (pixel[removedChannel] + pixel[1]) / 2;
            }
        }
        int rowLength = filteredData[0].length;
        int halfImageSize = filteredData.length * rowLength;

        int colorThreshold = 250, lastLeftCount = 0, lastRightCount = 0;
        while (colorThreshold > 50) {
            int leftCount = 0, rightCount = 0;
            for (double[] row : filteredData) {
                for (int i = 0; i < rowLength / 2; i++) {
                    if (row[i] > colorThreshold) leftCount++;
                }
                for (int i = rowLength / 2; i < rowLength; i++) {
                    if (row[i] > colorThreshold) rightCount++;
                }
            }

            if (leftCount > 0.05 * halfImageSize && leftCount > 1.5 * lastLeftCount) return 0;
            if (rightCount > 0.05 * halfImageSize && rightCount > 1.5 * lastRightCount) return 1;

            colorThreshold -= 5;
        }

        return 2;
    }
}
