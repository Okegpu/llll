package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Ball Color Detect")
public class Colordetectortest extends LinearOpMode {

    private ColorSensor color;
    private DistanceSensor distance;

    // ---- Tune these if needed ----
    private final double MAX_DIST_CM = 6.0;     // only classify when ball is close
    private final int SAMPLES = 5;              // averaging for stability

    // HSV gating (ignore low-quality readings)
    private final float MIN_SAT = 0.25f;
    private final float MIN_VAL = 0.15f;

    // Hue ranges (degrees 0..360)
    private final float GREEN_HUE_MIN = 80f;
    private final float GREEN_HUE_MAX = 165f;

    private final float PURPLE_HUE_MIN = 200f;
    private final float PURPLE_HUE_MAX = 240f;

    @Override
    public void runOpMode() {
        color = hardwareMap.get(ColorSensor.class, "color");
        distance = hardwareMap.get(DistanceSensor.class, "color");

        // REV Color Sensor V3 LED
        color.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {

            double distCm = distance.getDistance(DistanceUnit.CM);

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
            // Scale to 0..255 using the max channel to reduce dependence on brightness
            double max = Math.max(r, Math.max(g, b));
            int R = (int) Math.round(255.0 * (r / (max + 1e-9)));
            int G = (int) Math.round(255.0 * (g / (max + 1e-9)));
            int B = (int) Math.round(255.0 * (b / (max + 1e-9)));

            float[] hsv = new float[3];
            Color.RGBToHSV(R, G, B, hsv);
            float hue = hsv[0]; // 0..360
            float sat = hsv[1]; // 0..1
            float val = hsv[2]; // 0..1

            String detected;

            if (distCm > MAX_DIST_CM) {
                detected = "TOO FAR";
            } else if (sat < MIN_SAT || val < MIN_VAL) {
                detected = "UNSURE (LOW SAT/VAL)";
            } else if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
                detected = "GREEN BALL";
            } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
                detected = "PURPLE BALL";
            } else {
                detected = "OTHER";
            }

            // Telemetry (use this to tune hue ranges if needed)
            telemetry.addData("Dist (cm)", "%.1f", distCm);
            telemetry.addData("RGB avg", "%.0f %.0f %.0f", r, g, b);
            telemetry.addData("RGB scaled", "%d %d %d", R, G, B);
            telemetry.addData("Hue", "%.0f", hue);
            telemetry.addData("Sat Val", "%.2f %.2f", sat, val);
            telemetry.addData("Detected", detected);
            telemetry.update();
        }
    }
}
