package org.firstinspires.ftc.teamcode.nobles.findblock;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlockFinderPipeline extends OpenCvPipeline {
    private final boolean useRed;
    private int blockPosition = -1;
    private int colorThreshold = 255;

    public BlockFinderPipeline(boolean useRed) {
        this.useRed = useRed;
    }

    public int getBlockPosition() {
        return blockPosition;
    }

    @Override
    public Mat processFrame(Mat frame) {
        int targetChannel = useRed ? 0 : 2;
        int removedChannel = useRed ? 2 : 0;

        double[][] filteredData = new double[frame.height()][frame.width()];
        for (int y = 0; y < frame.height(); y++) {
            for (int x = 0; x < frame.width(); x++) {
                double[] pixel = frame.get(y, x);
                filteredData[y][x] = pixel[targetChannel] - (pixel[removedChannel] + pixel[1]) / 2;
            }
        }

        int thirdWidth = frame.width() / 3;
        int leftCount = countGreaterThan(filteredData, colorThreshold, 0, thirdWidth);
        int middleCount = countGreaterThan(filteredData, colorThreshold, thirdWidth, 2 * thirdWidth);
        int rightCount = countGreaterThan(filteredData, colorThreshold, 2 * thirdWidth, frame.width());

        int[] maxMidArray = argMaxAndMid(leftCount, middleCount, rightCount);
        int maxIndex = maxMidArray[0];
        int midIndex = maxMidArray[1];
        double[] counts = new double[] {leftCount, middleCount, rightCount};

        double confidence = 0;
        if (counts[maxIndex] > 0) confidence = (counts[maxIndex] - counts[midIndex]) / counts[maxIndex];

        if (confidence > 0.5 || colorThreshold < 50) {
            blockPosition = maxIndex;
            colorThreshold += 10;
        }
        else colorThreshold -= 10;

        Mat grayscaleMat = new Mat(frame.height(), frame.width(), CvType.CV_8U);
        for (int y = 0; y < frame.height(); y++) {
            for (int x = 0; x < frame.width(); x++) {
                grayscaleMat.put(y, x, filteredData[y][x]);
            }
        }
        return grayscaleMat;
    }

    private int countGreaterThan(double[][] data, double threshold, int x0, int x1) {
        int sum = 0;
        for (int y = 0; y < data.length; y++) {
            for (int x = x0; x < x1; x++) {
                if (data[y][x] > threshold) sum++;
            }
        }
        return sum;
    }

    private int[] argMaxAndMid(int a0, int a1, int a2) {
        int[] arr = new int[] {a0, a1, a2};
        int maxIndex = argMax(arr);
        arr[maxIndex] = -1;
        int midIndex = argMax(arr);
        return new int[] {maxIndex, midIndex};
    }

    private int argMax(int[] arr) {
        int maxIndex = 0;
        for (int i = 1; i < arr.length; i++) {
            if (arr[i] > arr[maxIndex]) maxIndex = i;
        }
        return maxIndex;
    }
}
