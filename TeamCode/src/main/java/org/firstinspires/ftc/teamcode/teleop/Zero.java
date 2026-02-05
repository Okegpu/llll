package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo To Zero")
public class Zero extends OpMode {

    private Servo indexer;

    @Override
    public void init() {
        indexer = hardwareMap.get(Servo.class, "angle"); // CHANGE NAME IF NEEDED
        indexer.setPosition(0); // servo to 0
    }

    @Override
    public void loop() {
    }
}
