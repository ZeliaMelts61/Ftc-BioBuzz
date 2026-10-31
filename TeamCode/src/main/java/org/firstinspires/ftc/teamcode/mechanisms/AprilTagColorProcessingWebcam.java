package org.firstinspires.ftc.teamcode.mechanisms;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

public class AprilTagColorProcessingWebcam {

    public enum Mode {
        APRIL_TAG,
        COLOR_BLOB,
        APRIL_TAG_AND_COLOR_BLOB,
        OFF
    }

    private AprilTagProcessor aprilTagProcessor;
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal visionPortal;
    private List<AprilTagClusterDetection> detectedTagClusters = new ArrayList<>();
    private List<ColorBlobLocatorProcessor.Blob> detectedBlobs = new ArrayList<>();

    private Mode mode;
//    private List<AprilTagSingleDetection> detectedSingle = new ArrayList<>();

    public AprilTagColorProcessingWebcam(Position cameraPosition, YawPitchRollAngles orientation){
        setMode(Mode.OFF);
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, orientation)
                .build();

        this.colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

    }

    public void init(HardwareMap hardwareMap, String webcamDeviceName){
        WebcamName webcam = hardwareMap.get(WebcamName.class, webcamDeviceName);
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTagProcessor)
                .addProcessor(colorLocator)
                .build();
    }

    public void update(){
        detectedTagClusters.clear();
        detectedBlobs.clear();
        if (visionPortal.getProcessorEnabled(aprilTagProcessor)) {
            updateAprilTags();
        }
        if (visionPortal.getProcessorEnabled(colorLocator)){
            updateColorBlob();
        }
    }

    private void updateAprilTags(){
        ArrayList<AprilTagDetection> detectedTags = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection) {
                detectedTagClusters.add((AprilTagClusterDetection) detection);
            }
        }
    }

    private void updateColorBlob(){
        detectedBlobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, detectedBlobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.6, 1, detectedBlobs);     /* filter out non-circular blobs.
         * NOTE: You may want to adjust the minimum value depending on your use case.
         * Circularity values will be affected by shadows, and will therefore vary based
         * on the location of the camera on your robot and venue lighting. It is strongly
         * encouraged to test your vision on the competition field if your event allows
         * sensor calibration time.
         */
        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                SortOrder.DESCENDING,
                detectedBlobs);
    }

    public List<AprilTagClusterDetection> getDetectedTagClusters(){
        return detectedTagClusters;
    }

    public AprilTagClusterDetection getClusterByName(String name){
        for(AprilTagClusterDetection cluster : detectedTagClusters){
            if(cluster.metadata.name.equalsIgnoreCase(name)){
                return cluster;
            }
        }
        return null;
    }

    public List<ColorBlobLocatorProcessor.Blob> getDetectedBlobs(){
        return detectedBlobs;
    }

    public ColorBlobLocatorProcessor.Blob getLargestDetectedBlob(){
        try {
            return detectedBlobs.get(0);
        } catch (IndexOutOfBoundsException e) {
            return null;
        }
    }

    public void stop(){
        if(visionPortal != null){
            visionPortal.close();
        }
    }

    public void pause(){
        if(visionPortal != null){
            visionPortal.stopStreaming();
            visionPortal.stopLiveView();
        }
    }

    public void unpause(){
        if(visionPortal != null){
            visionPortal.resumeStreaming();
            visionPortal.resumeLiveView();
        }
    }

    public void setMode(Mode mode){
        if (mode!=this.mode){
            if(mode.equals(Mode.APRIL_TAG_AND_COLOR_BLOB)){
                visionPortal.setProcessorEnabled(colorLocator, true);
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                unpause();
            } else if (mode.equals(Mode.APRIL_TAG)) {
                visionPortal.setProcessorEnabled(aprilTagProcessor, true);
                visionPortal.setProcessorEnabled(colorLocator, false);
                unpause();
            } else if (mode.equals(Mode.COLOR_BLOB)) {
                visionPortal.setProcessorEnabled(colorLocator, true);
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                unpause();
            } else {
                visionPortal.setProcessorEnabled(colorLocator, false);
                visionPortal.setProcessorEnabled(aprilTagProcessor, false);
                pause();
            }
            this.mode = mode;
        }


    }
}
