package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * StarterBot TeleOp v6: same controls as v5, plus a shooter speed light.
 *
 * The launcher runs at 47% of the motor's full speed (v5 used 350 ticks/s).
 * Holding gamepad 2 A (Cross) shoots in TURBO at 50% instead. Each time turbo
 * starts, the Driver Station says "Turbo" and gamepad 2 rumbles (the Control
 * Hub has no speaker of its own).
 *
 * While shooting (right bumper or A held):
 *   launcher at or below 35% of full speed -> windmill stopped, lights RED
 *   launcher above 35%                     -> windmill feeds,   lights GREEN
 * If a shot drags the launcher back to 35% or below, the windmill stops and
 * the lights go RED again until it recovers. When the launcher is not
 * running, the lights are off. While turbo (A) is held the lights are BLUE
 * instead (the windmill still only feeds above 35%).
 *
 * "Lights" are the goBILDA RGB light on Expansion Hub 2 servo port 3 (not in
 * the robot config; driven directly), the Control Hub and Expansion Hub LEDs,
 * and gamepad 2's light bar (PS4/PS5-style pads only). When off, the hubs
 * show their normal status pattern.
 *
 * Gamepad 1: left stick drive/strafe, right stick turn.
 * Gamepad 2: right trigger intake, left trigger reverse,
 *            right bumper shoot (launcher, then windmill above 35%), left bumper eject,
 *            A (hold) turbo shoot at 50%.
 */
@TeleOp(name = "Mec BioBuzz StarterBot Teleop v6", group = "StarterBot")
public class BioBuzzStarterbotTeleopV6 extends OpMode {
    // Launcher runs at this fraction of the motor's full free speed.
    // v5 used a fixed 350 ticks/s, which is only ~12% of a goBILDA 5202.
    public static final double LAUNCHER_SPEED_FRACTION = 0.47;
    // Turbo (gamepad 2 A held) runs the launcher at this fraction instead.
    public static final double LAUNCHER_TURBO_SPEED_FRACTION = 0.50;
    private static final int TURBO_RUMBLE_MS = 300;
    // Launcher readings go to the match log (tag "TeleopV6") every 0.25 s while
    // shooting, in /sdcard/FIRST/matchlogs/Match-*-Mec_BioBuzz_StarterBot_Teleop_v6.txt.
    private static final String TAG = "TeleopV6";
    private static final double LOG_PERIOD_S = 0.25;
    public static final double LAUNCHER_EJECT_VELOCITY = 35;
    // The windmill feeds, and the lights turn green, above this fraction of full speed.
    public static final double WINDMILL_FEED_MIN_SPEED_FRACTION = 0.35;
    // goBILDA RGB Indicator Light (same positions as Light Test / Shooter Test).
    private static final String GOBILDA_LIGHT_HUB = "Expansion Hub 2";
    private static final int GOBILDA_LIGHT_PORT = 3;
    private static final double GOBILDA_OFF = 0.0;
    private static final double GOBILDA_RED = 0.279;
    private static final double GOBILDA_GREEN = 0.500;
    private static final double GOBILDA_BLUE = 0.611;
    private static final List<Blinker.Step> SOLID_GREEN =
            Collections.singletonList(new Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS));
    private static final List<Blinker.Step> SOLID_RED =
            Collections.singletonList(new Blinker.Step(Color.RED, 1, TimeUnit.SECONDS));
    private static final List<Blinker.Step> SOLID_BLUE =
            Collections.singletonList(new Blinker.Step(Color.BLUE, 1, TimeUnit.SECONDS));

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;
    private DcMotorEx launcher;
    private CRServo windmillServo;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;
    private List<LynxModule> hubs;
    private ServoController gobildaLightServos;
    private double launcherMaxVelocity;
    private double launcherNormalVelocity;
    private double launcherTurboVelocity;
    private double launcherTargetVelocity;
    private double feedMinVelocity;
    private boolean turbo;
    private boolean lastTurbo;
    private final ElapsedTime logTimer = new ElapsedTime();

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;
    private double intakePower;
    private double intakeServoPower;
    private double windmillPower;

    private enum Light { OFF, RED, GREEN, BLUE }
    // null until the first loop so the lights are always set once.
    private Light currentLight = null;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        launcher = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        hubs = hardwareMap.getAll(LynxModule.class);
        gobildaLightServos = hardwareMap.tryGet(ServoController.class, GOBILDA_LIGHT_HUB);
        if (gobildaLightServos == null) {
            telemetry.addLine("Note: no '" + GOBILDA_LIGHT_HUB + "' servo controller; goBILDA light disabled.");
        }

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[] {leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40.0, 0.0, 0.0, 12.5));
        // Full free speed from the motor type in the robot config (goBILDA 5202: ~2800 ticks/s).
        launcherMaxVelocity = launcher.getMotorType().getMaxRPM() * launcher.getMotorType().getTicksPerRev() / 60.0;
        launcherNormalVelocity = LAUNCHER_SPEED_FRACTION * launcherMaxVelocity;
        launcherTurboVelocity = LAUNCHER_TURBO_SPEED_FRACTION * launcherMaxVelocity;
        launcherTargetVelocity = launcherNormalVelocity;
        RobotLog.ii(TAG, "init max=%.0f normal=%.0f turbo=%.0f ticks/s", launcherMaxVelocity,
                launcherNormalVelocity, launcherTurboVelocity);
        feedMinVelocity = WINDMILL_FEED_MIN_SPEED_FRACTION * launcherMaxVelocity;

        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
        windmillServo.setPower(0.0);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad2.right_trigger > 0) {
            intakePower = gamepad2.right_trigger;
            intakeServoPower = -gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) {
            intakePower = -gamepad2.left_trigger;
            intakeServoPower = intakePower;
        } else {
            intakePower = 0.0;
            intakeServoPower = 0.0;
        }

        updateTurbo();
        launch();
        double launcherVelocity = launcher.getVelocity();
        boolean launcherFastEnough = launcherVelocity > feedMinVelocity;
        boolean shooting = gamepad2.right_bumper || turbo;
        showLight(!shooting ? Light.OFF : turbo ? Light.BLUE : launcherFastEnough ? Light.GREEN : Light.RED);
        if (shooting && logTimer.seconds() >= LOG_PERIOD_S) {
            logTimer.reset();
            RobotLog.ii(TAG, "turbo=%d rb=%d target=%.0f actual=%.0f power=%.2f", turbo ? 1 : 0,
                    gamepad2.right_bumper ? 1 : 0, launcherTargetVelocity, launcherVelocity, launcher.getPower());
        }

        setIntakePower(intakePower, intakeServoPower);

        if (shooting) {
            // Feed only above 35%; stops again whenever the launcher drops to 35% or below.
            windmillPower = launcherFastEnough ? -1.0 : 0.0;
        } else if (gamepad2.left_bumper) {
            windmillPower = 1.0;
        } else if (gamepad2.left_trigger > 0) {
            windmillPower = gamepad2.left_trigger;
        } else {
            windmillPower = 0.0;
        }
        windmillServo.setPower(windmillPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad2.left_trigger, gamepad2.right_trigger);
        telemetry.addData("Launcher", "%.0f / %.0f ticks/s (%.0f%% of max)", launcherVelocity, launcherTargetVelocity,
                100.0 * launcherVelocity / launcherMaxVelocity);
        telemetry.addData("Launcher setting", "%s: %.0f%% of %.0f ticks/s max (normal %.0f%%, turbo %.0f%% on gp2 A)",
                turbo ? "TURBO" : "normal", 100.0 * launcherTargetVelocity / launcherMaxVelocity, launcherMaxVelocity,
                LAUNCHER_SPEED_FRACTION * 100, LAUNCHER_TURBO_SPEED_FRACTION * 100);
        telemetry.addData("Lights", "%s (green above %.0f ticks/s = 35%%, blue in turbo)", currentLight, feedMinVelocity);
        telemetry.addData("Windmill feed", !shooting ? "-" : launcherFastEnough ? "FEEDING" : "WAITING FOR SPEED");
        telemetry.addData("Intake motor / servos / windmill", "%.2f / %.2f / %.2f", intakePower, intakeServoPower, windmillPower);
        telemetry.addData("Controls", "BIOBUZZ v6 - speed lights: goBILDA + hubs + gamepad 2");
    }

    @Override
    public void stop() {
        showLight(Light.OFF);
    }

    // Only sends LED commands when the color changes, to keep the loop fast.
    // RED/GREEN/BLUE are pushed on top of the hub's own status pattern and popped
    // for OFF, so the normal status blink shows whenever the light is off.
    private void showLight(Light light) {
        if (light == currentLight) {
            return;
        }
        boolean hadPattern = currentLight != null && currentLight != Light.OFF;
        currentLight = light;
        for (LynxModule hub : hubs) {
            if (hadPattern) {
                hub.popPattern();
            }
            if (light == Light.RED) {
                hub.pushPattern(SOLID_RED);
            } else if (light == Light.GREEN) {
                hub.pushPattern(SOLID_GREEN);
            } else if (light == Light.BLUE) {
                hub.pushPattern(SOLID_BLUE);
            }
        }
        if (light == Light.RED) {
            gamepad2.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (light == Light.GREEN) {
            gamepad2.setLedColor(0.0, 1.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if (light == Light.BLUE) {
            gamepad2.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(0.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gobildaLightServos != null) {
            gobildaLightServos.setServoPosition(GOBILDA_LIGHT_PORT,
                    light == Light.RED ? GOBILDA_RED : light == Light.GREEN ? GOBILDA_GREEN
                            : light == Light.BLUE ? GOBILDA_BLUE : GOBILDA_OFF);
        }
    }

    private void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void setIntakePower(double motorPower, double servoPower) {
        intake.setPower(motorPower);
        leftIntakeServo.setPower(servoPower);
        rightIntakeServo.setPower(servoPower);
    }

    // Turbo while A is held. The sound and rumble fire once per press (rising
    // edge), not every loop, so holding A doesn't repeat them.
    private void updateTurbo() {
        turbo = gamepad2.a;
        if (turbo && !lastTurbo) {
            telemetry.speak("Turbo");
            gamepad2.rumble(TURBO_RUMBLE_MS);
        }
        if (turbo != lastTurbo) {
            RobotLog.ii(TAG, "turbo %s (gamepad 2 A), target %.0f ticks/s", turbo ? "ON" : "OFF",
                    turbo ? launcherTurboVelocity : launcherNormalVelocity);
        }
        lastTurbo = turbo;
        launcherTargetVelocity = turbo ? launcherTurboVelocity : launcherNormalVelocity;
    }

    private void launch() {
        if (gamepad2.right_bumper || turbo) {
            launcher.setVelocity(launcherTargetVelocity);
        } else if (gamepad2.left_bumper) {
            launcher.setVelocity(-LAUNCHER_EJECT_VELOCITY);
        } else {
            launcher.setVelocity(0.0);
        }
    }
}
