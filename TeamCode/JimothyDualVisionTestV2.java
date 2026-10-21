package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Locale;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/*
 * Copy of "Jimothy Dual Vision Test" with the red nectar filters removed.
 *
 * Changes from the original:
 *  - Reads the Limelight pipeline's class ID filter at INIT. If any filter is
 *    set (it was "1", which only let pollen through), it clears it for this
 *    session so every class reaches the robot. The filter is shown on screen.
 *  - Shows EVERY Limelight detection with its class ID, not just exact label
 *    matches, so a misspelled label (e.g. "red_nector") is never hidden.
 *  - Recognizes game pieces by loose label matching: pollen, red nectar,
 *    blue nectar - any case, spaces/hyphens/underscores, "nectar" or "nector".
 *  - Lists every class seen during the run, and writes each new one to the
 *    match log (tag "DualVisionV2").
 *
 * The rear webcam AprilTag part is unchanged.
 */
@TeleOp(name = "Jimothy Dual Vision Test v2", group = "Diagnostics")
public class JimothyDualVisionTestV2 extends LinearOpMode {
    private static final int LIMELIGHT_PIPELINE = 0;
    private static final String LIMELIGHT_API = "http://172.29.0.1:5807";
    private static final String TAG = "DualVisionV2";
    private static final Pattern ID_FILTER = Pattern.compile("\"detector_idfilters\"\\s*:\\s*\"([^\"]*)\"");

    // label -> "id N, best conf" for every class seen this run
    private final TreeMap<String, String> seenClasses = new TreeMap<>();
    private String idFilterStatus = "unknown";

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        WebcamName rearWebcam = hardwareMap.get(WebcamName.class, "Webcam 1");
        AprilTagProcessor aprilTags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        VisionPortal webcamPortal = new VisionPortal.Builder()
                .setCamera(rearWebcam)
                .addProcessor(aprilTags)
                .build();

        telemetry.setMsTransmissionInterval(50);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();
        clearIdFilterIfSet();

        telemetry.addLine("Forward Limelight: pipeline 0 neural detector (all classes)");
        telemetry.addLine("Rear webcam: AprilTag detector");
        telemetry.addData("Limelight class ID filter", idFilterStatus);
        telemetry.addLine("Press START to monitor both cameras");
        telemetry.update();

        waitForStart();
        try {
            while (opModeIsActive()) {
                showLimelight(limelight);
                showAprilTags(webcamPortal, aprilTags);
                telemetry.update();
                idle();
            }
        } finally {
            limelight.stop();
            webcamPortal.close();
        }
    }

    private void showLimelight(Limelight3A limelight) {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Limelight", "pipeline %d (%s), %.0f FPS",
                status.getPipelineIndex(), status.getPipelineType(), status.getFps());
        telemetry.addData("Class ID filter", idFilterStatus);

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            telemetry.addLine("  No Limelight result yet");
            showSeenClasses();
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        int pollen = 0, red = 0, blue = 0;
        int n = 0;
        for (LLResultTypes.DetectorResult d : detections) {
            n++;
            String label = d.getClassName();
            String kind = gamePieceKind(label);
            if ("POLLEN".equals(kind)) pollen++;
            if ("RED NECTAR".equals(kind)) red++;
            if ("BLUE NECTAR".equals(kind)) blue++;
            telemetry.addData("  LL " + n, "%s [id %d] %s conf=%.0f%% tx=%+.1f ty=%+.1f area=%.2f",
                    kind, d.getClassId(), label, d.getConfidence() * 100,
                    d.getTargetXDegrees(), d.getTargetYDegrees(), d.getTargetArea());
            noteClass(label, d.getClassId(), d.getConfidence());
        }
        if (n == 0) {
            telemetry.addLine("  No Limelight detections");
        }
        telemetry.addData("Game pieces", "pollen %d, red nectar %d, blue nectar %d", pollen, red, blue);
        showSeenClasses();
    }

    private void showSeenClasses() {
        telemetry.addData("Classes seen this run", seenClasses.isEmpty() ? "none yet" : "");
        for (String label : seenClasses.keySet()) {
            telemetry.addData("    " + label, seenClasses.get(label));
        }
    }

    private void noteClass(String label, int id, double conf) {
        String key = label == null ? "(null)" : label;
        String existing = seenClasses.get(key);
        double best = conf;
        if (existing != null) {
            Matcher m = Pattern.compile("best (\\d+)%").matcher(existing);
            if (m.find()) best = Math.max(best, Integer.parseInt(m.group(1)) / 100.0);
        } else {
            RobotLog.ii(TAG, "new class seen: label='%s' id=%d conf=%.2f kind=%s", key, id, conf, gamePieceKind(label));
        }
        seenClasses.put(key, String.format(Locale.US, "id %d, best %.0f%%, %s", id, best * 100, gamePieceKind(label)));
    }

    // Loose match so spelling/format differences in the model's labels don't hide a piece.
    private String gamePieceKind(String label) {
        if (label == null) return "OTHER";
        String s = label.toLowerCase(Locale.US).replaceAll("[\\s_\\-]+", "");
        if (s.contains("pollen")) return "POLLEN";
        boolean nectar = s.contains("nectar") || s.contains("nector");
        if (nectar && s.contains("red")) return "RED NECTAR";
        if (nectar && s.contains("blue")) return "BLUE NECTAR";
        return "OTHER";
    }

    private void clearIdFilterIfSet() {
        try {
            String filter = readIdFilter();
            if (filter == null) {
                idFilterStatus = "couldn't read (Limelight API not reachable)";
                return;
            }
            if (filter.isEmpty()) {
                idFilterStatus = "none (all classes)";
                return;
            }
            // flush=0: live for this session only; the saved pipeline isn't changed.
            post("/update-pipeline?flush=0", "{\"detector_idfilters\":\"\"}");
            String after = readIdFilter();
            idFilterStatus = (after != null && after.isEmpty())
                    ? "was \"" + filter + "\", cleared for this run"
                    : "was \"" + filter + "\", CLEAR FAILED (now \"" + after + "\")";
            RobotLog.ii(TAG, "class ID filter: %s", idFilterStatus);
        } catch (Exception e) {
            idFilterStatus = "error: " + e.getMessage();
            RobotLog.ee(TAG, e, "class ID filter check failed");
        }
    }

    private String readIdFilter() throws Exception {
        String json = get("/pipeline-atindex?index=" + LIMELIGHT_PIPELINE);
        if (json == null) return null;
        Matcher m = ID_FILTER.matcher(json);
        return m.find() ? m.group(1) : "";
    }

    private String get(String path) throws Exception {
        HttpURLConnection c = (HttpURLConnection) new URL(LIMELIGHT_API + path).openConnection();
        c.setConnectTimeout(1000);
        c.setReadTimeout(1000);
        try {
            return c.getResponseCode() == 200 ? readAll(c) : null;
        } finally {
            c.disconnect();
        }
    }

    private void post(String path, String body) throws Exception {
        HttpURLConnection c = (HttpURLConnection) new URL(LIMELIGHT_API + path).openConnection();
        c.setConnectTimeout(1000);
        c.setReadTimeout(1000);
        c.setRequestMethod("POST");
        c.setDoOutput(true);
        c.setRequestProperty("Content-Type", "application/json");
        try (OutputStream out = c.getOutputStream()) {
            out.write(body.getBytes(StandardCharsets.UTF_8));
        }
        try {
            c.getResponseCode();
        } finally {
            c.disconnect();
        }
    }

    private String readAll(HttpURLConnection c) throws Exception {
        StringBuilder sb = new StringBuilder();
        try (BufferedReader r = new BufferedReader(new InputStreamReader(c.getInputStream(), StandardCharsets.UTF_8))) {
            String line;
            while ((line = r.readLine()) != null) sb.append(line);
        }
        return sb.toString();
    }

    private void showAprilTags(VisionPortal portal, AprilTagProcessor processor) {
        telemetry.addData("Rear webcam", "%s", portal.getCameraState());
        List<AprilTagDetection> detections = processor.getDetections();
        telemetry.addData("AprilTags", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String clusterName = cluster.metadata == null ? "unknown cluster" : cluster.metadata.name;
                String name = cluster.metadata != null ? cluster.metadata.shortName : "unknown";
                if (detection.ftcPose == null) {
                    telemetry.addData("  Cluster " + name, "%s, %d%% visible", clusterName, cluster.percentClusterFound);
                } else {
                    telemetry.addData("  Cluster " + name, "%s, %d%% visible, range=%.1fin bearing=%+.1f yaw=%+.1f",
                            clusterName, cluster.percentClusterFound,
                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw);
                }
            } else {
                AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
                String name = single.metadata != null ? single.metadata.name : "unknown";
                if (detection.ftcPose == null) {
                    telemetry.addData("  Tag " + single.id, "%s", name);
                } else {
                    telemetry.addData("  Tag " + single.id, "%s range=%.1fin bearing=%+.1f yaw=%+.1f",
                            name, detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw);
                }
            }
        }
    }
}
