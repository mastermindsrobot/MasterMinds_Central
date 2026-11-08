
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.os.SystemClock;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.google.mediapipe.framework.image.BitmapImageBuilder;
import com.google.mediapipe.framework.image.MPImage;
import com.google.mediapipe.tasks.components.containers.Category;
import com.google.mediapipe.tasks.core.BaseOptions;
import com.google.mediapipe.tasks.vision.core.RunningMode;
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizer;
import com.google.mediapipe.tasks.vision.gesturerecognizer.GestureRecognizerResult;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;

@TeleOp(name = "MasterMinds Gesture Drive", group = "MasterMinds")
public class GesturesCode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private VisionPortal visionPortal;
    private HandGestureProcessor gestureProcessor;

    // Keep the robot slow during testing.
    private static final double TURN_POWER = 0.15;
    private static final double STRAFE_MULTIPLIER = 1.1;

    private static final String MODEL_ASSET_NAME =
            "gesture_recognizer.task";

    private static final long MAX_FRAME_AGE_MS = 500;

    @Override
    public void runOpMode() {

        try {
            // -----------------------------------------------
            // 1. Initialize drivetrain
            // -----------------------------------------------

            frontLeft = hardwareMap.get(DcMotor.class, "front_left");
            rightFront = hardwareMap.get(DcMotor.class, "front_right");
            leftBack = hardwareMap.get(DcMotor.class, "back_left");
            rightBack = hardwareMap.get(DcMotor.class, "back_right");

            // Preserve the directions from the working TeleOp.
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            stopDrive();

            // -----------------------------------------------
            // 2. Initialize MediaPipe and webcam
            // -----------------------------------------------

            gestureProcessor =
                    new HandGestureProcessor(hardwareMap.appContext);

            visionPortal = new VisionPortal.Builder()
                    .setCamera(
                            hardwareMap.get(
                                    WebcamName.class,
                                    "Webcam 1"
                            )
                    )
                    .setCameraResolution(new Size(320, 240))
                    .addProcessor(gestureProcessor)
                    .build();

            telemetry.addLine("MasterMinds Gesture Drive");
            telemetry.addLine("Closed fist: rotate clockwise");
            telemetry.addLine("Open palm: stop");
            telemetry.addLine("Other gesture / no hand: stop");
            telemetry.addLine(" ");
            telemetry.addLine("Waiting for start...");
            telemetry.update();

            waitForStart();

            // -----------------------------------------------
            // 3. Main control loop
            // -----------------------------------------------

            while (opModeIsActive()) {

                long now = SystemClock.uptimeMillis();

                long frameAge = now
                        - gestureProcessor.getLastFrameTimeMs();

                boolean cameraFresh =
                        frameAge >= 0
                                && frameAge <= MAX_FRAME_AGE_MS;

                boolean handFound =
                        cameraFresh
                                && gestureProcessor.isHandDetected();

                String rawGesture =
                        gestureProcessor.getRawGesture();

                String controlGesture =
                        gestureProcessor.getStableGesture();

                double forward = 0.0;
                double strafe = 0.0;
                double turn = 0.0;

                String action = "STOP";

                // Default to stopping unless a valid command exists.
                if (!cameraFresh) {

                    action = "STOP: CAMERA STALE";

                } else if (!handFound) {

                    action = "STOP: NO HAND";

                } else if ("Closed_Fist".equals(controlGesture)) {

                    turn = TURN_POWER;
                    action = "ROTATE CLOCKWISE";

                } else if ("Open_Palm".equals(controlGesture)) {

                    action = "STOP: OPEN PALM";

                } else if ("ACQUIRING_FIST".equals(controlGesture)) {

                    action = "STOP: CONFIRMING FIST";

                } else {

                    action = "STOP: UNRECOGNIZED";
                }

                driveRobotCentric(forward, strafe, turn);

                // -----------------------------------------------
                // 4. Diagnostic telemetry
                // -----------------------------------------------

                telemetry.addData(
                        "Camera Frames",
                        cameraFresh ? "RECEIVING" : "STALE / MISSING"
                );

                telemetry.addData(
                        "Frame Age",
                        frameAge + " ms"
                );

                telemetry.addData(
                        "Hand Detected",
                        handFound ? "YES" : "NO"
                );

                telemetry.addData(
                        "Raw Model Gesture",
                        rawGesture
                );

                telemetry.addData(
                        "Control Gesture",
                        controlGesture
                );

                telemetry.addData(
                        "Confidence",
                        "%.3f",
                        gestureProcessor.getConfidence()
                );

                telemetry.addData(
                        "Inference Age",
                        gestureProcessor.getInferenceAgeMs() + " ms"
                );

                telemetry.addData(
                        "Robot Action",
                        action
                );

                String error = gestureProcessor.getErrorMessage();

                if (error != null && !error.isEmpty()) {
                    telemetry.addData("Vision Error", error);
                }

                telemetry.update();

                idle();
            }

        } catch (Exception e) {

            stopDrive();

            telemetry.addLine("GESTURE TELEOP ERROR");
            telemetry.addData(
                    "Exception",
                    e.getClass().getSimpleName()
            );
            telemetry.addData(
                    "Details",
                    String.valueOf(e.getMessage())
            );
            telemetry.update();

        } finally {

            // Stop the motors before shutting down vision.
            stopDrive();

            if (visionPortal != null) {
                try {
                    visionPortal.close();
                } catch (Exception ignored) {
                    // Continue cleanup.
                }
            }

            if (gestureProcessor != null) {
                try {
                    gestureProcessor.close();
                } catch (Exception ignored) {
                    // Continue cleanup.
                }
            }
        }
    }

    // -----------------------------------------------
    // 5. Robot-centric mecanum drive
    // -----------------------------------------------

    private void driveRobotCentric(
            double forward,
            double strafe,
            double turn) {

        strafe *= STRAFE_MULTIPLIER;

        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double denominator = Math.max(
                Math.abs(forward)
                        + Math.abs(strafe)
                        + Math.abs(turn),
                1.0
        );

        frontLeft.setPower(fl / denominator);
        leftBack.setPower(bl / denominator);
        rightFront.setPower(fr / denominator);
        rightBack.setPower(br / denominator);
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0.0);
        if (rightFront != null) rightFront.setPower(0.0);
        if (leftBack != null) leftBack.setPower(0.0);
        if (rightBack != null) rightBack.setPower(0.0);
    }

    // ===============================================
    // 6. MediaPipe gesture recognizer
    // ===============================================

    private static class HandGestureProcessor
            implements VisionProcessor {

        private final GestureRecognizer recognizer;

        private Bitmap bitmap;
        private final Mat rgbaFrame = new Mat();

        // These values are read by the main OpMode thread.
        private volatile boolean handDetected = false;

        private volatile String rawGesture = "NO_HAND";
        private volatile String stableGesture = "NO_HAND";

        private volatile float confidence = 0.0f;

        private volatile long lastFrameTimeMs = 0L;
        private volatile long lastInferenceTimeMs = 0L;

        private volatile String errorMessage = "";

        // Inference approximately 8 times per second.
        private static final long INFERENCE_INTERVAL_MS = 120L;

        // Confidence required for an actionable gesture.
        private static final float MIN_GESTURE_CONFIDENCE = 0.60f;

        // A fist must appear in two consecutive inferences.
        private static final int REQUIRED_FIST_FRAMES = 2;

        private String candidateGesture = "";
        private int candidateCount = 0;

        HandGestureProcessor(Context context) throws IOException {

            String modelPath = copyModelToInternalStorage(context);

            BaseOptions baseOptions = BaseOptions.builder()
                    .setModelAssetPath(modelPath)
                    .build();

            GestureRecognizer.GestureRecognizerOptions options =
                    GestureRecognizer.GestureRecognizerOptions.builder()
                            .setBaseOptions(baseOptions)
                            .setRunningMode(RunningMode.VIDEO)
                            .setNumHands(1)

                            // Allow moderate variations in lighting
                            // and camera positioning.
                            .setMinHandDetectionConfidence(0.50f)
                            .setMinHandPresenceConfidence(0.50f)
                            .setMinTrackingConfidence(0.50f)
                            .build();

            recognizer = GestureRecognizer.createFromOptions(
                    context,
                    options
            );
        }

        private static String copyModelToInternalStorage(
                Context context) throws IOException {

            File modelFile = new File(
                    context.getFilesDir(),
                    MODEL_ASSET_NAME
            );

            try (
                    InputStream input =
                            context.getAssets().open(MODEL_ASSET_NAME);

                    FileOutputStream output =
                            new FileOutputStream(modelFile, false)
            ) {
                byte[] buffer = new byte[8192];
                int bytesRead;

                while ((bytesRead = input.read(buffer)) != -1) {
                    output.write(buffer, 0, bytesRead);
                }

                output.flush();
            }

            if (!modelFile.isFile() || modelFile.length() == 0L) {
                throw new IOException(
                        "Model is missing or empty: "
                                + modelFile.getAbsolutePath()
                );
            }

            return modelFile.getAbsolutePath();
        }

        @Override
        public void init(
                int width,
                int height,
                CameraCalibration calibration) {

            bitmap = Bitmap.createBitmap(
                    width,
                    height,
                    Bitmap.Config.ARGB_8888
            );
        }

        @Override
        public Object processFrame(
                Mat frame,
                long captureTimeNanos) {

            long now = SystemClock.uptimeMillis();

            // Update this for every camera frame, including skipped frames.
            lastFrameTimeMs = now;

            if (frame == null || frame.empty()) {
                handDetected = false;
                rawGesture = "EMPTY_FRAME";
                stableGesture = "NO_HAND";
                confidence = 0.0f;
                errorMessage = "Camera supplied an empty frame";
                resetCandidate();
                return null;
            }

            // Reduce load on the Control Hub.
            if (now - lastInferenceTimeMs < INFERENCE_INTERVAL_MS) {
                return null;
            }

            lastInferenceTimeMs = now;

            try {

                // Ensure that the bitmap matches the camera resolution.
                if (bitmap == null
                        || bitmap.isRecycled()
                        || bitmap.getWidth() != frame.cols()
                        || bitmap.getHeight() != frame.rows()) {

                    if (bitmap != null && !bitmap.isRecycled()) {
                        bitmap.recycle();
                    }

                    bitmap = Bitmap.createBitmap(
                            frame.cols(),
                            frame.rows(),
                            Bitmap.Config.ARGB_8888
                    );
                }

                // Convert the camera frame into RGBA format.
                if (frame.channels() == 4) {

                    frame.copyTo(rgbaFrame);

                } else if (frame.channels() == 3) {

                    Imgproc.cvtColor(
                            frame,
                            rgbaFrame,
                            Imgproc.COLOR_RGB2RGBA
                    );

                } else if (frame.channels() == 1) {

                    Imgproc.cvtColor(
                            frame,
                            rgbaFrame,
                            Imgproc.COLOR_GRAY2RGBA
                    );

                } else {

                    throw new IllegalStateException(
                            "Unsupported image channels: "
                                    + frame.channels()
                    );
                }

                Utils.matToBitmap(rgbaFrame, bitmap);

                MPImage image =
                        new BitmapImageBuilder(bitmap).build();

                // The timestamp must increase for each video inference.
                GestureRecognizerResult result =
                        recognizer.recognizeForVideo(image, now);

                lastInferenceTimeMs = now;
                errorMessage = "";

                // -----------------------------------------------
                // A. Detect the hand independently of its label.
                // -----------------------------------------------

                boolean foundHand =
                        result != null
                                && result.landmarks() != null
                                && !result.landmarks().isEmpty();

                handDetected = foundHand;

                if (!foundHand) {

                    rawGesture = "NO_HAND";
                    stableGesture = "NO_HAND";
                    confidence = 0.0f;

                    resetCandidate();
                    return null;
                }

                // A hand exists, even if the gesture classifier
                // cannot assign a reliable label.
                rawGesture = "UNCLASSIFIED";
                stableGesture = "UNRECOGNIZED";
                confidence = 0.0f;

                List<List<Category>> hands = result.gestures();

                if (hands == null
                        || hands.isEmpty()
                        || hands.get(0) == null
                        || hands.get(0).isEmpty()) {

                    resetCandidate();
                    return null;
                }

                // Find the highest-confidence category explicitly.
                Category best = null;

                for (Category category : hands.get(0)) {
                    if (category != null
                            && (best == null
                            || category.score() > best.score())) {
                        best = category;
                    }
                }

                if (best == null) {
                    resetCandidate();
                    return null;
                }

                String name = best.categoryName();
                float score = best.score();

                confidence = score;

                rawGesture = name == null
                        ? "UNCLASSIFIED"
                        : name;

                // Never allow a low-confidence classification
                // to command movement.
                if (name == null
                        || score < MIN_GESTURE_CONFIDENCE) {

                    stableGesture = "UNRECOGNIZED";
                    resetCandidate();
                    return null;
                }

                // -----------------------------------------------
                // B. Open palm stops immediately.
                // -----------------------------------------------

                if ("Open_Palm".equals(name)) {

                    stableGesture = "Open_Palm";
                    resetCandidate();
                    return null;
                }

                // -----------------------------------------------
                // C. Require repeated fist detection before turning.
                // -----------------------------------------------

                if ("Closed_Fist".equals(name)) {

                    if (name.equals(candidateGesture)) {
                        candidateCount++;
                    } else {
                        candidateGesture = name;
                        candidateCount = 1;
                    }

                    if (candidateCount >= REQUIRED_FIST_FRAMES) {

                        stableGesture = "Closed_Fist";

                    } else {

                        // Stop while waiting for confirmation.
                        stableGesture = "ACQUIRING_FIST";
                    }

                    return null;
                }

                // Everything else is an unrecognized command
                // for this specific robot-control program.
                stableGesture = "UNRECOGNIZED";
                resetCandidate();

            } catch (Exception e) {

                // Inference errors are never treated as commands.
                handDetected = false;
                rawGesture = "INFERENCE_ERROR";
                stableGesture = "STOP";
                confidence = 0.0f;

                errorMessage = e.getClass().getSimpleName()
                        + ": "
                        + String.valueOf(e.getMessage());

                resetCandidate();
            }

            return null;
        }

        private void resetCandidate() {
            candidateGesture = "";
            candidateCount = 0;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext) {
            // Telemetry is used for diagnostics in this version.
        }

        boolean isHandDetected() {
            return handDetected;
        }

        String getRawGesture() {
            return rawGesture;
        }

        String getStableGesture() {
            return stableGesture;
        }

        float getConfidence() {
            return confidence;
        }

        long getLastFrameTimeMs() {
            return lastFrameTimeMs;
        }

        long getInferenceAgeMs() {
            long timestamp = lastInferenceTimeMs;

            if (timestamp == 0L) {
                return -1L;
            }

            return SystemClock.uptimeMillis() - timestamp;
        }

        String getErrorMessage() {
            return errorMessage;
        }

        void close() {
            recognizer.close();
            rgbaFrame.release();

            if (bitmap != null && !bitmap.isRecycled()) {
                bitmap.recycle();
                bitmap = null;
            }
        }
    }
}
