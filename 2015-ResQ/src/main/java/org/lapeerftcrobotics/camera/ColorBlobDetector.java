package org.lapeerftcrobotics.camera;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ColorBlobDetector {
    // Lower and Upper bounds for range checking in HSV color space
    private Scalar mLowerBound = new Scalar(0);
    private Scalar mUpperBound = new Scalar(0);
    private Scalar mLowerBound2 = new Scalar(0);
    private Scalar mUpperBound2 = new Scalar(0);
    // Minimum contour area in percent for contours filtering
    private static double mMinContourArea = 0.1;
    // Color radius for range checking in HSV color space
    private Scalar mColorRadius = new Scalar(25,50,50,0);
//    private Mat mSpectrum = new Mat();
    private List<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

    private String hsvString = "";

    // Cache
    Mat mPyrDownMat = new Mat();
    Mat mHsvMat = new Mat();
    Mat mPreDilatedMask = new Mat();
    Mat mDilatedMask = new Mat();

    Mat mSubRegionRgba = new Mat();
    Mat mSubRegionHsv = new Mat();
    Mat mMask = new Mat();
    Mat mMask2 = new Mat();
    Mat mMask3 = new Mat();
    Mat mHierarchy = new Mat();

    Scalar hsvColor = new Scalar(0,0,0,0);
    Rect subMatRect = new Rect();
    Point subMatP1 = new Point();
    Point subMatP2 = new Point();
    Mat subMat = new Mat();
    Scalar drawColor = new Scalar(0,0,0,0);
//    double[] contourAreas = new double[0];
    Rect[] rBounding = new Rect[3];

    public void setLowerBounds( Scalar lower) {
        mLowerBound.val[0] = lower.val[0];
        mLowerBound.val[1] = lower.val[1];
        mLowerBound.val[2] = lower.val[2];
        mLowerBound.val[3] = 0;
    }

    public void setUpperBounds( Scalar upper) {
        mUpperBound.val[0] = upper.val[0];
        mUpperBound.val[1] = upper.val[1];
        mUpperBound.val[2] = upper.val[2];
        mUpperBound.val[3] = 255;
    }

    public void setLowerBounds2( Scalar lower) {
        mLowerBound2.val[0] = lower.val[0];
        mLowerBound2.val[1] = lower.val[1];
        mLowerBound2.val[2] = lower.val[2];
        mLowerBound2.val[3] = 0;
    }

    public void setUpperBounds2( Scalar upper) {
        mUpperBound2.val[0] = upper.val[0];
        mUpperBound2.val[1] = upper.val[1];
        mUpperBound2.val[2] = upper.val[2];
        mUpperBound2.val[3] = 255;
    }

    public Scalar getHsvColor() {
        return hsvColor;
    }

    public void setSubMatRect(Rect r) {
        this.subMatRect = r;
        this.subMatP1 = new Point(subMatRect.x,subMatRect.y);
        this.subMatP2 = new Point(subMatRect.x+subMatRect.width,subMatRect.y+subMatRect.height);
    }

    public Rect[] getBoundingRectangles() {
        return this.rBounding;
    }

    public void setDrawColor(Scalar c) {
        this.drawColor = c;
    }

    public void setHsvColor(Scalar hsvColor) {
        double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
        double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 255) ? hsvColor.val[0]+mColorRadius.val[0] : 255;

        mLowerBound.val[0] = minH;
        mUpperBound.val[0] = maxH;

        mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
        mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

        mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
        mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

        mLowerBound.val[3] = 0;
        mUpperBound.val[3] = 255;
    }

    public void setMinContourArea(double area) {
        mMinContourArea = area;
    }

    public String getHsvString() {
        return this.hsvString;
    }

    public void process(Mat mRgba) {
        process(mRgba, false, false);
    }

    public void process(Mat mRgba, boolean useMultiBounds) { process(mRgba, useMultiBounds, false); }

    public void process(Mat mRgba, boolean useMultiBounds, boolean logHSV) {

        Mat mSubRegionRgba = mRgba.submat(this.subMatRect);
        Mat mSubRegionHsv = new Mat();
        Imgproc.cvtColor(mSubRegionRgba, mSubRegionHsv, Imgproc.COLOR_RGB2HSV_FULL);

        // Calculate average color of touched region
        hsvColor = Core.sumElems(mSubRegionHsv);
        int pointCount = subMatRect.width*subMatRect.height;
        for (int i = 0; i < hsvColor.val.length; i++)
            hsvColor.val[i] /= pointCount;
/*
        hsvString = "";
        StringBuffer sb = new StringBuffer();
        for(int i=0; i<subMatRect.width-1; i++) {
            for(int j=0; j<subMatRect.height-1; j++) {
                Mat sub = mSubRegionHsv.submat(new Rect(i, j, 1, 1));
                Scalar s = Core.sumElems(sub);
                sb.append("[").append(i).append("][").append(j).append("] ");
                sb.append("H:").append(s.val[0]).append(" S:").append(s.val[1]).append(" V:").append(s.val[2]);
                sb.append("\r\n");
            }
            sb.append("\r\n");
        }
        hsvString = sb.toString();
*/
        Core.inRange(mSubRegionHsv, mLowerBound, mUpperBound, mMask);

        if (useMultiBounds) {
            Core.inRange(mSubRegionHsv, mLowerBound2, mUpperBound2, mMask2);
            Core.bitwise_or(mMask, mMask2, mMask3);
            Imgproc.dilate(mMask3, mPreDilatedMask, new Mat()); // dilate after masked inrange
        } else {
            Imgproc.dilate(mMask, mPreDilatedMask, new Mat()); // dilate after masked inrange
        }

        Imgproc.dilate(mPreDilatedMask, mDilatedMask, new Mat()); // dilate after masked inrange
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> c = contours.iterator();
        while (c.hasNext()) {
            MatOfPoint cont = c.next();
            double area = Imgproc.contourArea(cont);

            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        c = contours.iterator();
        while (c.hasNext()) {
            MatOfPoint contour = c.next();
            if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                mContours.add(contour);
            }
        }

        Core.rectangle(mSubRegionRgba, new Point(1,1), new Point(this.subMatRect.width-1,this.subMatRect.height-1), this.drawColor, 1);

        Imgproc.drawContours(mSubRegionRgba, mContours, -1, this.drawColor, 1);

        rBounding[0] = null;
        rBounding[1] = null;
        rBounding[2] = null;

        for(int i=0; i<mContours.size(); i++) {
            if (i<3) {
                Rect r = Imgproc.boundingRect(mContours.get(i));
                rBounding[i] = r;
                Core.rectangle(mSubRegionRgba, r.br(), r.tl(), this.drawColor, 1);
                int x = r.x + r.width / 2;
                int y = r.y + r.height / 2;
                Core.line(mSubRegionRgba, new Point(x, y - 2), new Point(x, y + 2), this.drawColor, 1);
                Core.line(mSubRegionRgba, new Point(x - 2, y), new Point(x + 2, y), this.drawColor, 1);
            }
        }
    }
/*
    public void processOriginal(Mat rgbaImage) {

        Imgproc.pyrDown(rgbaImage, mPyrDownMat);
        Imgproc.pyrDown(mPyrDownMat, mPyrDownMat);

        Imgproc.cvtColor(mPyrDownMat, mHsvMat, Imgproc.COLOR_RGB2HSV_FULL);

        Core.inRange(mHsvMat, mLowerBound, mUpperBound, mMask);

        Imgproc.dilate(mMask, mDilatedMask, new Mat());

        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

        Imgproc.findContours(mDilatedMask, contours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find max contour area
        double maxArea = 0;
        Iterator<MatOfPoint> each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint wrapper = each.next();
            double area = Imgproc.contourArea(wrapper);
            if (area > maxArea)
                maxArea = area;
        }

        // Filter contours by area and resize to fit the original image size
        mContours.clear();
        each = contours.iterator();
        while (each.hasNext()) {
            MatOfPoint contour = each.next();
            if (Imgproc.contourArea(contour) > mMinContourArea*maxArea) {
                Core.multiply(contour, new Scalar(4,4), contour);
                mContours.add(contour);
            }
        }
    }
*/
    public List<MatOfPoint> getContours() {
        return mContours;
    }
}
