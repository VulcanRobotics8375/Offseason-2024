package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/* TODO:
*   1. test the code
*   2. add PID? or odometry
* */

@TeleOp(name = "simpleDrive")
public class simpleDrive extends OpMode {
    // declaration
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    BNO055IMU imu; // imu embedded into the control hub

    @Override
    public void init(){
        // initialization
        fl = hardwareMap.get(DcMotor.class,"front_left");
        fr = hardwareMap.get(DcMotor.class,"front_right");
        bl = hardwareMap.get(DcMotor.class,"back_left");
        br = hardwareMap.get(DcMotor.class,"back_right");

        // run w/o using encoder; we're controlling it with a joystick + don't need
        // constant velocity to be maintained
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // left side motors = forward, right side = reversed
        // this way, they'll both work in the same direction
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /* NOTE: up is negative for y stick + down is positive
    *  both sticks range from -1 to 1
    *  left stick's x axis = controls direction
    *  right stick's y axis = controls power
    * */
    public void loop(){
        // movement code
        double y = -gamepad1.right_stick_y; // sets power to value left stick is at
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x; // rx = right x

        // not sure if these are the actual values yet lol, can figure out w/ test cases?
        fl.setPower(y + rx + x);
        fr.setPower(y - rx - x);
        bl.setPower(y + rx - x);
        br.setPower(y - rx + x);
    }

    @Override
    public void stop() {
        super.stop();
    }
}
