package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * TeleOp program for a mecanum drive robot with a shooter motor.
 * Runs continuously while the OpMode is active.
 */
@TeleOp(name = "Turret", group = "Turret")
public class TurretTesting extends OpMode {
    private DcMotor shootMotor;  // shooter

    @Override
    public void init() {
        // Hardware mapping
        shootMotor = hardwareMap.get(DcMotor.class, "in_motor");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Shooter always on (full power)
        shootMotor.setPower(1.0);
    }

}


