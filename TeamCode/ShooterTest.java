package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Spins shooter_motor and, once it's fast enough, the windmill feeder.
 * Intake and drive motors never move.
 *
 * Windmill (windmill_servo, Control Hub servo port 2), while the shooter runs:
 *   shooter at or below 35% of full speed -> windmill stopped
 *   shooter above 35%                     -> windmill feeds
 * It stops again whenever the shooter drops back to 35% or below.
 *
 * goBILDA RGB light on Expansion Hub 2 servo port 3 (not in the robot
 * config; driven directly), while the shooter is running:
 *   speed at or below 35% of full speed -> RED
 *   speed above 35%                     -> GREEN
 * It's off when the shooter is off.
 *
 * The Control Hub and Expansion Hub LEDs and gamepad 1's light bar
 * (PS4/PS5-style pads only) are GREEN whenever the shooter is running.
 *
 * Gamepad 1:
 *   A          - toggle shooter on/off
 *   B          - stop shooter
 *   D-pad up   - target speed +25 ticks/s
 *   D-pad down - target speed -25 ticks/s
 *
 * Encoder readings are written to the match log (tag "ShooterTest")
 * every 0.25 s, in /sdcard/FIRST/matchlogs/Match-*-Shooter_Test.txt.
 */
@TeleOp(name = "Shooter Test", group = "Diagnostics")
public class ShooterTest extends OpMode {
    // Starts at 45% of full speed, the same as TeleOp v6.
    private static final double DEFAULT_SPEED_FRACTION = 0.45;
    // TeleOp v6 starts the windmill (and turns its light green) at this fraction.
    private static final double FEED_MIN_SPEED_FRACTION = 0.40;
    private static final double READY_TOLERANCE_FRACTION = 0.03;
    private static final double STEP = 25.0;
    private static final double LOG_INTERVAL_SECONDS = 0.25;
    private static final String TAG = "ShooterTest";
    private static final List<Blinker.Step> SOLID_GREEN =
            Collections.singletonList(new Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS));
    // goBILDA RGB Indicator Light (same positions as Light Test).
    private static final String GOBILDA_LIGHT_HUB = "Expansion Hub 2";
    private static final int GOBILDA_LIGHT_PORT = 3;
    private static final double GOBILDA_OFF = 0.0;
    private static final double GOBILDA_RED = 0.279;
    private static final double GOBILDA_GREEN = 0.500;
    // goBILDA light turns green, and the windmill feeds, above this fraction of full speed.
    private static final double GOBILDA_GREEN_SPEED_FRACTION = 0.35;
    // Same feed direction/power as TeleOp v5/v6.
    private static final double WINDMILL_FEED_POWER = -1.0;

    private DcMotorEx shooter;
    private CRServo windmillServo;
    private List<LynxModule> hubs;
    private ServoController gobildaLightServos;
    private double gobildaPosition = -1.0;
    private boolean lightOn = false;
    private double maxVelocity;
    private double targetVelocity;
    private boolean running = false;
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();

    private boolean lastA, lastUp, lastDown;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40.0, 0.0, 0.0, 12.5));
        shooter.setVelocity(0.0);
        maxVelocity = shooter.getMotorType().getMaxRPM() * shooter.getMotorType().getTicksPerRev() / 60.0;
        targetVelocity = DEFAULT_SPEED_FRACTION * maxVelocity;
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setPower(0.0);
        hubs = hardwareMap.getAll(LynxModule.class);
        gobildaLightServos = hardwareMap.tryGet(ServoController.class, GOBILDA_LIGHT_HUB);
        if (gobildaLightServos == null) {
            telemetry.addLine("Note: no '" + GOBILDA_LIGHT_HUB + "' servo controller; goBILDA light disabled.");
        }
        telemetry.addLine("Shooter Test ready. Press START, then A to spin.");
        RobotLog.ii(TAG, "init encoderPos=%d", shooter.getCurrentPosition());
    }

    @Override
    public void start() {
        runTimer.reset();
        logTimer.reset();
        setGobildaLight(GOBILDA_OFF);
        RobotLog.ii(TAG, "columns: t_s, on, target_tps, actual_tps, encoderPos, power");
    }

    @Override
    public void loop() {
        boolean wasRunning = running;
        if (gamepad1.a && !lastA) {
            running = !running;
        }
        if (gamepad1.b) {
            running = false;
        }
        if (running != wasRunning) {
            RobotLog.ii(TAG, "shooter %s at t=%.2f", running ? "ON" : "OFF", runTimer.seconds());
        }
        if (gamepad1.dpad_up && !lastUp) {
            targetVelocity = Range.clip(targetVelocity + STEP, 0.0, maxVelocity);
        }
        if (gamepad1.dpad_down && !lastDown) {
            targetVelocity = Range.clip(targetVelocity - STEP, 0.0, maxVelocity);
        }
        lastA = gamepad1.a;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        shooter.setVelocity(running ? targetVelocity : 0.0);
        setLight(running);

        double velocity = shooter.getVelocity();
        boolean aboveGreenSpeed = velocity > GOBILDA_GREEN_SPEED_FRACTION * maxVelocity;
        setGobildaLight(!running ? GOBILDA_OFF : aboveGreenSpeed ? GOBILDA_GREEN : GOBILDA_RED);
        boolean feeding = running && aboveGreenSpeed;
        windmillServo.setPower(feeding ? WINDMILL_FEED_POWER : 0.0);
        boolean ready = running && Math.abs(velocity - targetVelocity) <= READY_TOLERANCE_FRACTION * targetVelocity;
        if (logTimer.seconds() >= LOG_INTERVAL_SECONDS) {
            logTimer.reset();
            RobotLog.ii(TAG, "%.2f, %d, %.0f, %.0f, %d, %.2f", runTimer.seconds(), running ? 1 : 0,
                    targetVelocity, velocity, shooter.getCurrentPosition(), shooter.getPower());
        }
        telemetry.addData("Shooter", running ? "ON" : "OFF");
        telemetry.addData("goBILDA light", "%s (green above %.0f ticks/s = 35%%)",
                !running ? "off" : aboveGreenSpeed ? "GREEN" : "RED", GOBILDA_GREEN_SPEED_FRACTION * maxVelocity);
        telemetry.addData("Hub / gamepad light", lightOn ? "GREEN" : "off");
        telemetry.addData("Windmill", !running ? "off" : feeding ? "FEEDING" : "WAITING FOR 35%");
        telemetry.addData("Target", "%.0f ticks/s (%.0f%% of %.0f max)", targetVelocity,
                100.0 * targetVelocity / maxVelocity, maxVelocity);
        telemetry.addData("Actual", "%.0f ticks/s (%s)", velocity, ready ? "READY" : running ? "SPINNING UP" : "STOPPED");
        telemetry.addData("Speed", "%.0f%% of max", 100.0 * velocity / maxVelocity);
        telemetry.addData("v6 feed point (40%)", "%s (%.0f ticks/s)",
                velocity >= FEED_MIN_SPEED_FRACTION * maxVelocity ? "REACHED - v6 would feed" : "below",
                FEED_MIN_SPEED_FRACTION * maxVelocity);
        telemetry.addData("Motor power", "%.2f", shooter.getPower());
        telemetry.addLine("A: on/off   B: stop   D-pad: speed");
    }

    @Override
    public void stop() {
        shooter.setVelocity(0.0);
        windmillServo.setPower(0.0);
        setLight(false);
        setGobildaLight(GOBILDA_OFF);
        RobotLog.ii(TAG, "stop at t=%.2f encoderPos=%d", runTimer.seconds(), shooter.getCurrentPosition());
    }

    // Green is pushed on top of the hub's own status pattern and popped when
    // the shooter stops. Only sends commands when the state changes.
    private void setLight(boolean on) {
        if (on == lightOn) {
            return;
        }
        lightOn = on;
        for (LynxModule hub : hubs) {
            if (on) {
                hub.pushPattern(SOLID_GREEN);
            } else {
                hub.popPattern();
            }
        }
        if (on) {
            gamepad1.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad1.setLedColor(0.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    // Only sends a command when the color changes.
    private void setGobildaLight(double position) {
        if (gobildaLightServos != null && position != gobildaPosition) {
            gobildaPosition = position;
            gobildaLightServos.setServoPosition(GOBILDA_LIGHT_PORT, position);
        }
    }
}
