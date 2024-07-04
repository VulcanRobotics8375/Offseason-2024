package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="helloWorld")
public class helloWorld extends OpMode{

    // instantiates motor as DcMotor type
    public DcMotor motor;

    @Override
    public void init()
    {
        // retrieves DcMotor from current configuration
        motor = hardwareMap.get(DcMotor.class, "motor");
        telemetry.addLine("motor has been initialized! ready for button");
        /*
        int testing = 5000;
        telemetry.addLine("hello world!");
        telemetry.addData("erm what the scallop!",testing);*/
    }

    @Override
    public void loop()
    {
        // gamepad 1a button (green(?), right middle bottom)
        // should only turn on motor if button is pressed
        if(gamepad1.a) {
            motor.setPower(0.5);
        }
        // else statement, or leave it as is?
        motor.setPower(0);
    }
    // TODO: PID exercise using motor?
}