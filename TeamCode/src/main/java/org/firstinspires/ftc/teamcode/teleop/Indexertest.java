package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Indexer", group="Drive")
public class Indexertest extends LinearOpMode {

    // =========================
    // HARDWARE
    // =========================
    private Servo indexer;
    private ColorSensor color;
    private DistanceSensor distance;

    // Optional intake
    private DcMotor intake;
    private static final boolean INTAKE_ENABLED = false;
    private static final double INTAKE_POWER = 1.0; // change if you enable intake

    // =========================
    // SERVO POSITIONS (TUNE)
    // =========================
    private final double[] SLOT_POS = {
            0.20,  // Slot 1 (HOME)
            0.38,  // Slot 2
            0.59   // Slot 3
    };

    // Servo busy lockout (ignore detection while moving)
    private static final long MOVE_LOCKOUT_MS = 250;
    private long ignoreDetectUntilMs = 0;

    // =========================
    // HSV + DISTANCE DETECTION (reliable)
    // =========================
    private static final double MAX_DIST_CM = 6.0;   // must be <= this to classify/store
    private static final double LEAVE_DIST_CM = 8.0; // must be > this to re-arm (hysteresis)
    private static final int SAMPLES = 5;            // sample averaging

    private static final float MIN_SAT = 0.25f;
    private static final float MIN_VAL = 0.15f;

    private static final float GREEN_HUE_MIN = 80f;
    private static final float GREEN_HUE_MAX = 165f;

    private static final float PURPLE_HUE_MIN = 200f;
    private static final float PURPLE_HUE_MAX = 345f;



    public enum BallColor { EMPTY, GREEN, PURPLE }

    private final BallColor[] slotColor = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };
    private final boolean[] slotFull = { false, false, false };
    private int nextFillSlot = 0;     // fills 0 -> 1 -> 2
    private boolean armed = true;     // only store when armed (re-arms when ball leaves)

    @Override
    public void runOpMode() {

        indexer = hardwareMap.get(Servo.class, "indexer");

        // REV Color Sensor V3 provides both interfaces under the SAME config name
        color = hardwareMap.get(ColorSensor.class, "colory");
        distance = hardwareMap.get(DistanceSensor.class, "colory");

        if (INTAKE_ENABLED) intake = hardwareMap.get(DcMotor.class, "intake");

        // LED on (REV V3 supports)
        color.enableLed(true);

        clearAll();
        nextFillSlot = 0;

        // Start at slot 1
        goToSlot(0);

        telemetry.addLine("READY: Indexer (HSV + Distance)");
        telemetry.addLine("Only GREEN/PURPLE stores. Slot1->Slot2->Slot3. Slot3 filled = no move.");
        telemetry.addLine("X = reset all, A/B/Y = force slot 1/2/3 (debug)");
        telemetry.update();

        waitForStart();

        // Force slot 1 on start
        goToSlot(0);
        sleep(150);

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();
            boolean servoBusy = now < ignoreDetectUntilMs;

            double distCm = distance.getDistance(DistanceUnit.CM);
            boolean distOk = Double.isFinite(distCm);

            // Re-arm when ball leaves (and we have a valid distance)
            if (!armed && distOk && distCm > LEAVE_DIST_CM) {
                armed = true;
            }

            // Optional intake stop if full
            if (INTAKE_ENABLED && intake != null) {
                intake.setPower(isFull() ? 0.0 : INTAKE_POWER);
            }

            // Manual debug: move servo to slots (proves positions work)
            if (gamepad1.a) { goToSlot(0); sleep(180); }
            if (gamepad1.b) { goToSlot(1); sleep(180); }
            if (gamepad1.y) { goToSlot(2); sleep(180); }

            // Reset all
            if (gamepad1.x) {
                clearAll();
                nextFillSlot = 0;
                armed = true;
                goToSlot(0);
                sleep(250);
            }

            BallColor detected = BallColor.EMPTY;

            // Detect ONLY if: not moving, armed, not full, and distance valid
            if (!servoBusy && armed && !isFull() && distOk && distCm <= MAX_DIST_CM) {
                detected = detectBallHSV();
                if (detected == BallColor.GREEN || detected == BallColor.PURPLE) {
                    storeBallAndAdvance(detected);
                    armed = false; // require leave before storing again
                }
            }

            telemetry.addData("ServoBusy", servoBusy);
            telemetry.addData("Dist(cm)", distOk ? String.format("%.1f", distCm) : "NaN");
            telemetry.addData("armed", armed);
            telemetry.addData("detected", detected);

            telemetry.addData("slot1", "%s | full=%s", slotColor[0], slotFull[0]);
            telemetry.addData("slot2", "%s | full=%s", slotColor[1], slotFull[1]);
            telemetry.addData("slot3", "%s | full=%s", slotColor[2], slotFull[2]);
            telemetry.addData("FULL", isFull());

            telemetry.update();
        }
    }

    // =========================
    // DETECT COLOR USING HSV (your working approach)
    // Returns EMPTY if unsure
    // =========================
    private BallColor detectBallHSV() {
        // Average RGB samples (reduces flicker)
        double rSum = 0, gSum = 0, bSum = 0;
        for (int i = 0; i < SAMPLES; i++) {
            rSum += color.red();
            gSum += color.green();
            bSum += color.blue();
            sleep(10);
        }

        double r = rSum / SAMPLES;
        double g = gSum / SAMPLES;
        double b = bSum / SAMPLES;

        // Convert to HSV
        // Use SUM normalization for stability (raw RGB can be in 1000s and that's fine)
        double sum = r + g + b;
        if (sum < 1e-6) return BallColor.EMPTY;

        int R = (int) Math.round(255.0 * (r / (sum + 1e-9)));
        int G = (int) Math.round(255.0 * (g / (sum + 1e-9)));
        int B = (int) Math.round(255.0 * (b / (sum + 1e-9)));

        float[] hsv = new float[3];
        Color.RGBToHSV(R, G, B, hsv);
        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];

        // Gate low-quality reads
        if (sat < MIN_SAT || val < MIN_VAL) return BallColor.EMPTY;

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) return BallColor.GREEN;
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) return BallColor.PURPLE;

        return BallColor.EMPTY;
    }

    // =========================
    // STORE + ADVANCE
    // =========================
    private void storeBallAndAdvance(BallColor sensed) {
        if (isFull()) return;

        // Ensure nextFillSlot points to an empty slot
        while (nextFillSlot < 3 && slotFull[nextFillSlot]) nextFillSlot++;
        if (nextFillSlot >= 3) return;

        // Store into that slot
        slotColor[nextFillSlot] = sensed;
        slotFull[nextFillSlot] = true;

        // If we just filled slot 3, do NOT move anymore
        if (nextFillSlot == 2) {
            nextFillSlot++;
            return;
        }

        // Move to the next slot position to be ready for next ball
        nextFillSlot++;
        goToSlot(nextFillSlot);
        sleep(150);
    }

    // =========================
    // HELPERS
    // =========================
    private void goToSlot(int slotIndex) {
        slotIndex = Math.max(0, Math.min(2, slotIndex));
        indexer.setPosition(SLOT_POS[slotIndex]);
        ignoreDetectUntilMs = System.currentTimeMillis() + MOVE_LOCKOUT_MS;
    }

    private boolean isFull() {
        return slotFull[0] && slotFull[1] && slotFull[2];
    }

    private void clearAll() {
        for (int i = 0; i < 3; i++) {
            slotColor[i] = BallColor.EMPTY;
            slotFull[i] = false;
        }
    }
}
