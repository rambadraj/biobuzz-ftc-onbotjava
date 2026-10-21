package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Tests a goBILDA RGB Indicator Light on Expansion Hub 2, servo port 3.
 *
 * The light isn't in the robot config, so this drives the port directly
 * through Expansion Hub 2's servo controller. No servos are configured on
 * the Expansion Hub, so nothing else is affected.
 *
 * Gamepad 1:
 *   A                - toggle auto-cycle through all colors (1 s each)
 *   D-pad right/left - next / previous color
 *   Bumpers R/L      - fine-tune position +/- 0.01
 *   B                - light off
 */
@TeleOp(name = "Light Test", group = "Diagnostics")
public class LightTest extends OpMode {
    private static final String LIGHT_HUB = "Expansion Hub 2";
    private static final int LIGHT_PORT = 3;
    private static final double CYCLE_SECONDS = 1.0;

    // goBILDA RGB Indicator Light color positions (SDK default 600-2400 us range).
    private static final String[] COLOR_NAMES = {
        "Off", "Red", "Orange", "Yellow", "Sage", "Green", "Azure", "Blue", "Indigo", "Violet", "White"
    };
    private static final double[] COLOR_POSITIONS = {
        0.0, 0.279, 0.333, 0.388, 0.444, 0.500, 0.555, 0.611, 0.666, 0.722, 1.0
    };

    private ServoController hubServos;

    private int colorIndex = 1;
    private double position = COLOR_POSITIONS[1];
    private boolean autoCycle = true;
    private final ElapsedTime cycleTimer = new ElapsedTime();

    private boolean lastA, lastRight, lastLeft, lastRB, lastLB;

    @Override
    public void init() {
        // Every hub gets a servo controller named after the hub, even with no servos configured.
        hubServos = hardwareMap.tryGet(ServoController.class, LIGHT_HUB);
        if (hubServos == null) {
            telemetry.addLine("ERROR: no servo controller named '" + LIGHT_HUB + "'. Found: "
                    + hardwareMap.getAllNames(ServoController.class));
        } else {
            telemetry.addLine("Light Test ready (" + LIGHT_HUB + " servo port " + LIGHT_PORT + "). Press START.");
        }
    }

    @Override
    public void start() {
        cycleTimer.reset();
        if (hubServos != null) {
            setColor(colorIndex);
        }
    }

    @Override
    public void loop() {
        if (hubServos == null) {
            telemetry.addLine("ERROR: '" + LIGHT_HUB + "' servo controller not found. Found: "
                    + hardwareMap.getAllNames(ServoController.class));
            return;
        }
        if (gamepad1.a && !lastA) {
            autoCycle = !autoCycle;
            cycleTimer.reset();
        }
        if (gamepad1.dpad_right && !lastRight) {
            autoCycle = false;
            setColor((colorIndex + 1) % COLOR_NAMES.length);
        }
        if (gamepad1.dpad_left && !lastLeft) {
            autoCycle = false;
            setColor((colorIndex + COLOR_NAMES.length - 1) % COLOR_NAMES.length);
        }
        if (gamepad1.right_bumper && !lastRB) {
            autoCycle = false;
            setPosition(position + 0.01);
        }
        if (gamepad1.left_bumper && !lastLB) {
            autoCycle = false;
            setPosition(position - 0.01);
        }
        if (gamepad1.b) {
            autoCycle = false;
            setColor(0);
        }
        lastA = gamepad1.a;
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
        lastRB = gamepad1.right_bumper;
        lastLB = gamepad1.left_bumper;

        if (autoCycle && cycleTimer.seconds() >= CYCLE_SECONDS) {
            cycleTimer.reset();
            // Skip "Off" while cycling so the light is always visibly on.
            setColor(colorIndex % (COLOR_NAMES.length - 1) + 1);
        }

        telemetry.addData("Port", "%s servo %d", LIGHT_HUB, LIGHT_PORT);
        telemetry.addData("Mode", autoCycle ? "AUTO-CYCLE" : "MANUAL");
        telemetry.addData("Color", nearestColorName());
        telemetry.addData("Position", "%.3f (~%.0f us)", position, 600 + position * 1800);
        telemetry.addLine("A: auto   D-pad L/R: color   Bumpers: fine   B: off");
        telemetry.addLine("Light dark? Check the cable is in " + LIGHT_HUB + " servo port " + LIGHT_PORT
                + " and the battery is charged.");
    }

    @Override
    public void stop() {
        if (hubServos != null) {
            hubServos.setServoPosition(LIGHT_PORT, 0.0);
        }
    }

    private void setColor(int index) {
        colorIndex = index;
        setPosition(COLOR_POSITIONS[index]);
    }

    private void setPosition(double newPosition) {
        position = Range.clip(newPosition, 0.0, 1.0);
        hubServos.setServoPosition(LIGHT_PORT, position);
    }

    private String nearestColorName() {
        int best = 0;
        for (int i = 1; i < COLOR_POSITIONS.length; i++) {
            if (Math.abs(COLOR_POSITIONS[i] - position) < Math.abs(COLOR_POSITIONS[best] - position)) {
                best = i;
            }
        }
        return COLOR_NAMES[best];
    }
}
