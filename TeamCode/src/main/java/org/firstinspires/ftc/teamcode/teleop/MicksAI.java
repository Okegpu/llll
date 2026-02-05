package org.firstinspires.ftc.teamcode.teleop;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.List;

@Config
@TeleOp(name="MicksAI - ULTIMATE (Fixed)", group="Drive")
public class MicksAI extends LinearOpMode {

    // =========================================================
    // *** DASHBOARD VARIABLES (TUNE LIVE) ***
    // =========================================================

    // 1. PHYSICS BOOSTERS
    // Lowered slightly to prevent overshooting at long range
    public static double EFFICIENCY_FACTOR = 1.36;

    // Lowered from 300 -> 150 (The Sweet Spot)
    public static double FLAT_RPM_BOOST    = 150.0;

    // 2. GEOMETRY
    public static double CAMERA_HEIGHT_INCHES = 13.0;
    public static double CAMERA_ANGLE_DEG     = 25.0;
    public static double SHOOTER_ANGLE_DEG    = 55.0;
    public static double SHOOTER_HEIGHT_INCHES= 16.0;
    public static double TARGET_HEIGHT_INCHES = 42.0;

    // 3. CONSTANTS
    public static double GRAVITY_IN_SEC2      = 386.09;
    public static double WHEEL_DIAMETER_IN    = 3.78;

    // 4. INDEXER POSITIONS
    public static double[] SLOT_POS  = {0.20, 0.38, 0.59};
    public static double[] SHOOT_POS = {0.50, 0.68, 0.89};

    // 5. KICKER POSITIONS
    public static double KICKER_DOWN = 1.00;
    public static double KICKER_UP   = 0.35;

    // =========================================================

    // --- HARDWARE ---
    private DcMotor fL, fR, bL, bR;
    private DcMotor evil; // Intake
    private Servo indexer, kicker;
    private ColorSensor color;
    private DistanceSensor distance;
    private Limelight3A limelight;
    private DcMotorEx turret, shooter1, shooter2;

    // --- CONSTANTS ---
    private static final double STRAFE_SCALE = 1.10;
    private static final double TX_kP = 0.025;
    private static final double TX_kD = 0.002;

    // Turret Math for Telemetry
    private static final double MOTOR_OUT_TICKS_PER_REV = 383.6;
    private static final double EXTRA_GEAR_RATIO = 100.0 / 54.0;
    private static final double TICKS_PER_TURRET_REV = MOTOR_OUT_TICKS_PER_REV * EXTRA_GEAR_RATIO;
    private static final int TURRET_ZERO_OFFSET_TICKS = 0;

    // --- SHOOTER MODES ---
    private static final double VEL_SOFT=1000, VEL_MED=1900, VEL_STRONG=3000;
    private enum ShotMode { SOFT, MEDIUM, STRONG }
    private ShotMode manualMode = ShotMode.MEDIUM;

    // --- STATE VARIABLES ---
    private double manualOffset = 0;
    private boolean evilOn = false, lastLB = false;
    private boolean lastRB = false, lastDup = false, lastDdown = false, lastDright = false;
    private long lastValidTagMs = 0;

    // --- INDEXER LOGIC ---
    private boolean indexerBusy = false;
    private long indexerBusyUntilMs = 0;
    private long ignoreDetectUntilMs = 0;
    private double lastIndexerPos = 0.20;

    // --- SORTER ---
    public enum BallColor { EMPTY, GREEN, PURPLE }
    private final BallColor[] slotColor = {BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY};
    private final boolean[] slotFull = {false, false, false};
    private int nextFillSlot = 0;
    private boolean armed = true, intakeEnabled = true;
    private BallColor lastDet = BallColor.EMPTY;
    private int detCount = 0;
    private long lastStoreMs = 0;

    // --- LOGIC ---
    private enum ShootState { IDLE, INDEX_TO_SHOOT, KICK_UP, HOLD, KICK_DOWN }
    private ShootState shootState = ShootState.IDLE;
    private long shootTimer = 0;
    private int shootingSlot = -1;
    private boolean clearMem = true;
    private boolean magDumpActive = false;
    private int manualSlot = 0;

    // --- RESET ---
    private enum ResetState { NONE, KICK_DOWN, INDEX_TO_NEXT }
    private ResetState resetState = ResetState.NONE;
    private long resetTimer = 0;

    private double lastTx = 0;
    private long lastTxMs = 0;

    @Override
    public void runOpMode() {
        // Init Dashboard Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initHardware();

        telemetry.addLine("ULTIMATE MODE READY.");
        telemetry.addLine("Physics: Tuned Down (+150 RPM)");
        telemetry.update();

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            long now = System.currentTimeMillis();

            // 1. DRIVE
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * STRAFE_SCALE;
            double rx = -gamepad1.right_stick_x;
            double den = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));
            fL.setPower((y + x + rx) / den); fR.setPower((y - x - rx) / den);
            bL.setPower((y - x + rx) / den); bR.setPower((y + x - rx) / den);

            // 2. TARGETING
            LLResult res = limelight.getLatestResult();
            boolean valid = (res != null && res.isValid());
            if(valid) {
                List results = res.getFiducialResults();
                if(results == null || results.size() == 0) valid = false;
            }

            if(valid) lastValidTagMs = now;

            double targetVel = VEL_MED;
            double dist = 0;

            if(valid) {
                // Get Distance
                double angleRad = Math.toRadians(CAMERA_ANGLE_DEG + res.getTy());
                dist = (TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / Math.tan(angleRad);

                // Calculate RPM (With Reduced Boost)
                targetVel = calculatePhysicsRPM(dist);

                // Lock Turret
                updateTurret(res.getTx(), now);
            } else {
                // Manual Fallback
                if(!gamepad1.back) {
                    if(gamepad1.dpad_up) manualMode = ShotMode.STRONG;
                    if(gamepad1.dpad_left) manualMode = ShotMode.MEDIUM;
                    if(gamepad1.dpad_down) manualMode = ShotMode.SOFT;
                }
                targetVel = (manualMode == ShotMode.STRONG) ? VEL_STRONG :
                        (manualMode == ShotMode.SOFT) ? VEL_SOFT : VEL_MED;
                turret.setPower(0);
            }

            // 3. LIVE TUNING + OFFSET
            if(gamepad1.back) {
                if(gamepad1.dpad_up && !lastDup) manualOffset += 50;
                if(gamepad1.dpad_down && !lastDdown) manualOffset -= 50;
            }

            // Combine Physics + Manual Offset + Flat Boost
            double finalVel = targetVel + manualOffset + FLAT_RPM_BOOST;

            shooter1.setVelocity(finalVel);
            shooter2.setVelocity(finalVel);

            // 4. MECHANISMS
            handleMechanisms(now);

            // 5. TELEMETRY
            boolean outtakeHeld = (gamepad1.left_trigger > 0.6);
            double distCm = distance.getDistance(DistanceUnit.CM);
            boolean distOk = Double.isFinite(distCm);

            telemetry.addData("ShotMode", manualMode);
            telemetry.addData("ManualNextSlot", (manualSlot+1));
            telemetry.addData("IntakeToggle", evilOn);
            telemetry.addData("OuttakeHeld", outtakeHeld);

            telemetry.addData("TargetVel", "%.0f", targetVel);
            telemetry.addData("Boost", "+%.0f", FLAT_RPM_BOOST);
            telemetry.addData("S1 vel", "%.0f", shooter1.getVelocity());

            telemetry.addData("TurretDeg", "%.1f", getTurretDeg());
            telemetry.addData("TagSeen", valid);
            telemetry.addData("Dist(in)", "%.1f", dist); // Added Distance in Inches

            telemetry.addData("Slots(mem)", "%s | %s | %s", slotColor[0],slotColor[1],slotColor[2]);
            telemetry.update();

            lastDup = gamepad1.dpad_up; lastDdown = gamepad1.dpad_down;
            lastDright = gamepad1.dpad_right; lastRB = gamepad1.right_bumper;
        }
    }

    // =========================================================
    // *** PHYSICS ENGINE (TUNED) ***
    // =========================================================
    private double calculatePhysicsRPM(double distanceInches) {
        double y = TARGET_HEIGHT_INCHES - SHOOTER_HEIGHT_INCHES;
        double x = distanceInches;
        double theta = Math.toRadians(SHOOTER_ANGLE_DEG);

        double term1 = x * Math.tan(theta) - y;

        if(term1 <= 0) return VEL_STRONG;

        double v_inches_sec = x / (Math.cos(theta) * Math.sqrt( (2 * term1) / GRAVITY_IN_SEC2 ));

        double circumference = Math.PI * WHEEL_DIAMETER_IN;
        double rpm = (v_inches_sec / circumference) * 60;

        return rpm * EFFICIENCY_FACTOR;
    }

    // =========================================================
    // *** SPINDEXER & MECHANISMS ***
    // =========================================================
    private void handleMechanisms(long now) {
        // INTAKE
        boolean out = (gamepad1.left_trigger > 0.6);
        if(gamepad1.left_bumper && !lastLB) evilOn = !evilOn;
        lastLB = gamepad1.left_bumper;

        if(out) { evil.setPower(-1.0); intakeEnabled=false; armed=false; }
        else { evil.setPower(evilOn ? 1.0 : 0); if(evilOn) intakeEnabled=true; }

        boolean busy = (now < ignoreDetectUntilMs) || indexerBusy ||
                (shootState != ShootState.IDLE) || (resetState != ResetState.NONE);

        boolean b=gamepad1.b, y=gamepad1.y, a=gamepad1.a, x=gamepad1.x;
        boolean rbClick = gamepad1.right_bumper && !lastRB;
        boolean dRightClick = gamepad1.dpad_right && !lastDright;

        // RESET BUTTON (X)
        if(x && !busy) startReset();

        // RESET INTAKE (A)
        if(a && !busy) { intakeEnabled=true; armed=true; magDumpActive=false; goToSlot(recalcNext()); }

        if(!busy && shootState == ShootState.IDLE) {
            // MAG DUMP (RB)
            if(rbClick) {
                int s = findFirstFull();
                if(s != -1) { magDumpActive=true; startShoot(s, true); }
            }
            // MANUAL CYCLE (D-Right)
            else if(dRightClick) {
                startShoot(manualSlot, false);
                manualSlot = (manualSlot + 1) % 3;
            }
            // SMART SHOOT (B=Green, Y=Purple)
            else if(!magDumpActive) {
                if(b) { int s = findColor(BallColor.GREEN); if(s!=-1) startShoot(s, true); }
                if(y) { int s = findColor(BallColor.PURPLE); if(s!=-1) startShoot(s, true); }
            }
        }

        // AUTO SORTER (3-Ball Logic)
        double d = distance.getDistance(DistanceUnit.CM);
        if(intakeEnabled && !armed && d > 8.0) { armed=true; detCount=0; }

        // Detect and Store
        if(intakeEnabled && !busy && armed && !isFull() && d <= 6.0) {
            BallColor c = detectColor();
            if(c != BallColor.EMPTY) {
                if(c == lastDet) detCount++; else { lastDet=c; detCount=1; }

                // If we see the same color 3 times, STORE IT
                if(detCount >= 3 && now-lastStoreMs > 350) {
                    storeBall(c);
                    lastStoreMs=now;
                    armed=false;
                    detCount=0;
                }
            } else detCount=0;
        }

        updateShoot(now);
        updateReset(now);
        if(indexerBusy && now >= indexerBusyUntilMs) indexerBusy = false;
    }

    // --- HELPERS ---

    private void goToSlot(int i) {
        if(i>=0 && i<3) setIndexer(SLOT_POS[i]);
    }

    private void updateTurret(double tx, long now) {
        if(Math.abs(tx) <= 1.0) { turret.setPower(0); return; }
        double dt = Math.max(0.001, (now-lastTxMs)/1000.0);
        double p = (TX_kP * tx) + (TX_kD * ((tx-lastTx)/dt));
        turret.setPower(Math.max(-0.9, Math.min(0.9, -p)));
        lastTx=tx; lastTxMs=now;
    }

    private double getTurretDeg(){
        int ticks = turret.getCurrentPosition() - TURRET_ZERO_OFFSET_TICKS;
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    private void startShoot(int slot, boolean clear) {
        shootingSlot=slot; clearMem=clear;
        shootState=ShootState.INDEX_TO_SHOOT; shootTimer=System.currentTimeMillis();
        intakeEnabled=false; setIndexer(SHOOT_POS[slot]);
    }

    private void updateShoot(long now) {
        switch(shootState) {
            case IDLE: return;
            case INDEX_TO_SHOOT:
                if(!indexerBusy) { kicker.setPosition(KICKER_UP); shootState=ShootState.KICK_UP; shootTimer=now; } break;
            case KICK_UP:
                if(now-shootTimer >= 350) { shootState=ShootState.HOLD; shootTimer=now; } break;
            case HOLD:
                if(now-shootTimer >= 140) { kicker.setPosition(KICKER_DOWN); shootState=ShootState.KICK_DOWN; shootTimer=now; } break;
            case KICK_DOWN:
                if(now-shootTimer >= 450) {
                    if(clearMem && shootingSlot >= 0) {
                        slotFull[shootingSlot]=false;
                        slotColor[shootingSlot]=BallColor.EMPTY;
                    }
                    shootState=ShootState.IDLE;
                    if(magDumpActive) {
                        int next = findFirstFull();
                        if(next != -1) startShoot(next, true); else magDumpActive=false;
                    }
                } break;
        }
    }

    private void startReset() {
        for(int i=0;i<3;i++){slotFull[i]=false; slotColor[i]=BallColor.EMPTY;}
        intakeEnabled=true; armed=true; detCount=0; magDumpActive=false;
        kicker.setPosition(KICKER_DOWN); resetState=ResetState.KICK_DOWN; resetTimer=System.currentTimeMillis();
    }

    private void updateReset(long now) {
        if(resetState==ResetState.KICK_DOWN && now-resetTimer>=250) {
            goToSlot(recalcNext()); resetState=ResetState.INDEX_TO_NEXT;
        } else if(resetState==ResetState.INDEX_TO_NEXT && !indexerBusy) resetState=ResetState.NONE;
    }

    private void storeBall(BallColor c) {
        recalcNext();
        if(nextFillSlot<0) return; // Full

        slotColor[nextFillSlot]=c;
        slotFull[nextFillSlot]=true;

        recalcNext();
        if(nextFillSlot>=0) goToSlot(nextFillSlot);
    }

    private void setIndexer(double pos) {
        indexer.setPosition(pos);
        long dur = (long)(90 + Math.abs(pos-lastIndexerPos)*500);
        lastIndexerPos=pos; indexerBusy=true;
        indexerBusyUntilMs=System.currentTimeMillis()+dur+180;
        ignoreDetectUntilMs=System.currentTimeMillis()+250;
    }

    private int recalcNext() { nextFillSlot=-1; for(int i=0;i<3;i++) if(!slotFull[i]){nextFillSlot=i;break;} return nextFillSlot; }
    private boolean isFull() { return slotFull[0]&&slotFull[1]&&slotFull[2]; }
    private int findFirstFull() { for(int i=0;i<3;i++) if(slotFull[i]) return i; return -1; }
    private int findColor(BallColor c) { for(int i=2;i>=0;i--) if(slotFull[i]&&slotColor[i]==c) return i; return -1; }

    private BallColor detectColor() {
        float[] hsv = new float[3];
        Color.RGBToHSV(color.red()*8, color.green()*8, color.blue()*8, hsv);
        if(hsv[0]>=80 && hsv[0]<=165) return BallColor.GREEN;
        if(hsv[0]>=200 && hsv[0]<=345) return BallColor.PURPLE;
        return BallColor.EMPTY;
    }

    private void initHardware() {
        fL=hardwareMap.get(DcMotor.class,"front_left"); fR=hardwareMap.get(DcMotor.class,"front_right");
        bL=hardwareMap.get(DcMotor.class,"back_left"); bR=hardwareMap.get(DcMotor.class,"back_right");
        fL.setDirection(DcMotorSimple.Direction.REVERSE); bL.setDirection(DcMotorSimple.Direction.REVERSE);

        evil=hardwareMap.get(DcMotor.class,"evil"); evil.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer=hardwareMap.get(Servo.class,"indexer"); kicker=hardwareMap.get(Servo.class,"kick");
        turret=hardwareMap.get(DcMotorEx.class,"turret"); turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1"); shooter2=hardwareMap.get(DcMotorEx.class,"shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER); shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        color=hardwareMap.get(ColorSensor.class,"color"); distance=hardwareMap.get(DistanceSensor.class,"color");
        color.enableLed(true); limelight=hardwareMap.get(Limelight3A.class,"limelight"); limelight.pipelineSwitch(0);

        kicker.setPosition(KICKER_DOWN); goToSlot(0);
    }
}