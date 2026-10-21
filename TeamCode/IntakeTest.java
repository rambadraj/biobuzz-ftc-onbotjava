package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * Runs the intake (intake_motor + left/right intake servos) as a unit.
 * The windmill feeder, shooter and drive motors are never powered.
 *
 * Gamepad 1:
 *   A          - toggle intake IN
 *   Y          - toggle EJECT (reverse)
 *   B          - stop
 *   X          - cycle parts: ALL -> MOTOR ONLY -> SERVOS ONLY
 *   D-pad up   - power +0.05
 *   D-pad down - power -0.05
 *
 * Directions match the autonomous: IN = motor +, servos -.
 * Note: intake_motor's encoder port is wired to the left odometry pod,
 * so its "velocity" is the odometry wheel, not the intake. Current is real.
 */
@TeleOp(name = "Intake Test", group = "Diagnostics")
public class IntakeTest extends OpMode {
    private static final double DEFAULT_POWER = 1.0;
    private static final double MIN_POWER = 0.1;
    private static final double STEP = 0.05;
    // goBILDA 5202 stalls around 9 A; sustained draw above this suggests a jam.
    private static final double JAM_CURRENT_AMPS = 6.0;

    private enum Direction { STOPPED, IN, EJECT }
    private enum Parts { ALL, MOTOR_ONLY, SERVOS_ONLY }

    private DcMotorEx intakeMotor;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    private Direction direction = Direction.STOPPED;
    private Parts parts = Parts.ALL;
    private double power = DEFAULT_POWER;
    private double peakCurrent = 0.0;

    private boolean lastA, lastY, lastX, lastUp, lastDown;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_intake_servo");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        stopAll();
        telemetry.addLine("Intake Test ready. Press START, then A to intake.");
    }

    @Override
    public void loop() {
        if (gamepad1.a && !lastA) {
            direction = direction == Direction.IN ? Direction.STOPPED : Direction.IN;
        }
        if (gamepad1.y && !lastY) {
            direction = direction == Direction.EJECT ? Direction.STOPPED : Direction.EJECT;
        }
        if (gamepad1.b) {
            direction = Direction.STOPPED;
        }
        if (gamepad1.x && !lastX) {
            parts = Parts.values()[(parts.ordinal() + 1) % Parts.values().length];
            peakCurrent = 0.0;
        }
        if (gamepad1.dpad_up && !lastUp) {
            power = Range.clip(power + STEP, MIN_POWER, 1.0);
        }
        if (gamepad1.dpad_down && !lastDown) {
            power = Range.clip(power - STEP, MIN_POWER, 1.0);
        }
        lastA = gamepad1.a;
        lastY = gamepad1.y;
        lastX = gamepad1.x;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        double sign = direction == Direction.IN ? 1.0 : direction == Direction.EJECT ? -1.0 : 0.0;
        double motorPower = parts != Parts.SERVOS_ONLY ? sign * power : 0.0;
        double servoPower = parts != Parts.MOTOR_ONLY ? -sign * power : 0.0;
        intakeMotor.setPower(motorPower);
        leftIntakeServo.setPower(servoPower);
        rightIntakeServo.setPower(servoPower);

        double current = intakeMotor.getCurrent(CurrentUnit.AMPS);
        peakCurrent = Math.max(peakCurrent, current);

        telemetry.addData("Intake", direction);
        telemetry.addData("Parts", parts);
        telemetry.addData("Power", "%.2f", power);
        telemetry.addData("Motor / servo command", "%+.2f / %+.2f", motorPower, servoPower);
        telemetry.addData("Motor current", "%.2f A (peak %.2f A)", current, peakCurrent);
        if (motorPower != 0.0 && current > JAM_CURRENT_AMPS) {
            telemetry.addLine("!! HIGH CURRENT - possible jam, press B");
        }
        telemetry.addLine("A: in   Y: eject   B: stop   X: parts   D-pad: power");
    }

    @Override
    public void stop() {
        stopAll();
    }

    private void stopAll() {
        intakeMotor.setPower(0.0);
        leftIntakeServo.setPower(0.0);
        rightIntakeServo.setPower(0.0);
    }
}
