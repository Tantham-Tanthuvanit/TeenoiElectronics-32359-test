package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TEST_TELEOP", group = "Test")
public class TEST extends LinearOpMode {

    private DcMotor fl; // front left
    private DcMotor fr; // front right
    private DcMotor rl; // rear left
    private DcMotor rr; // rear right

    double movementY;
    double movementX;
    double rotation;

    double MAX_TICKS_PER_SECOND = 2800;

    @Override
    public void runOpMode() throws InterruptedException {

        // Mapping hardware
        fl = hardwareMap.get(DcMotor.class, "FrontLeft");
        fr = hardwareMap.get(DcMotor.class, "FrontRight");
        rl = hardwareMap.get(DcMotor.class, "RearLeft");
        rr = hardwareMap.get(DcMotor.class, "RearRight");

        // main loop
        while (opModeIsActive()) {
            getInput();
            move();
        }

    }

    public void getInput() {
        // get movement for code
        movementY = -gamepad1.left_stick_y;
        movementX = gamepad1.left_stick_x;
        rotation = gamepad1.right_stick_x;
    }

    public void move() {
        // calculate powers for each motor
        double frontLeft    =   movementY + movementX + rotation;
        double rearLeft     =   movementY - movementX + rotation;
        double frontRight   =   movementY - movementX - rotation;
        double rearRight    =   movementY + movementX - rotation;

        // Normalizing
        double max = Math.abs(frontLeft);
        max = Math.max(max, Math.abs(frontRight));
        max = Math.max(max, Math.abs(rearLeft));
        max = Math.max(max, Math.abs(rearRight));
        max = Math.max(max, 1.0);

        // Apply normalized values
        frontLeft   /= max;
        rearLeft    /= max;
        frontRight  /= max;
        rearRight   /= max;
    }

}
