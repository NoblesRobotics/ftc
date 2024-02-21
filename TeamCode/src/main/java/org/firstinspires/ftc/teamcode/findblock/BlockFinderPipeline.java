package org.firstinspires.ftc.teamcode.findblock;

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

    // Called with every new frame
    @Override
    public Mat processFrame(Mat frame) {
        // Only run the time-consuming getBlockPosition function if we want to be capturing a frame
        if (isCapturing) {
            blockPosition = getBlockPosition(frame);
            isCapturing = false;
        }

        return frame;
    }

    private int getBlockPosition(Mat frame) {
        int targetChannel = useRed ? 0 : 2;
        int removedChannel = useRed ? 2 : 0;

        // Apply a filter to each pixel so as to capture only a strong intensity of the color we
        // are looking for, not white
        // A lot more about this in the portfolio
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
            // Count the pixels on each half of the screen that meet the color threshold
            int leftCount = 0, rightCount = 0;
            for (double[] row : filteredData) {
                for (int i = 0; i < rowLength / 2; i++) {
                    if (row[i] > colorThreshold) leftCount++;
                }
                for (int i = rowLength / 2; i < rowLength; i++) {
                    if (row[i] > colorThreshold) rightCount++;
                }
            }

            // If, after any lowering of the threshold, the number of pixels above the threshold is
            // at least 1.5x greater than before, we say that there is a box on that half of the
            // image (works based on the assumption that all pixels of the box will have roughly the
            // same intensity)
            if (leftCount > 0.05 * halfImageSize && leftCount > 1.5 * lastLeftCount) return 0;
            if (rightCount > 0.05 * halfImageSize && rightCount > 1.5 * lastRightCount) return 1;

            colorThreshold -= 5;
        }

        // The camera is pointed at the left and middle spike marks, so if it's not either of those,
        // the box is on the right
        return 2;
    }
}
