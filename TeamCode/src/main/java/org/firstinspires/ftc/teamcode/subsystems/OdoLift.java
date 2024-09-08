package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class OdoLift extends Subsystem {
    private Servo odoLiftMid, odoLiftLeft;

    private final double ODO_MID_DOWN = 0.01;
    private final double ODO_MID_UP = 0.8967;
    private final double ODO_LEFT_DOWN = 0.99;
    private final double ODO_LEFT_UP = 0.01;

    private double odoMidPos = ODO_MID_UP;
    private double odoLeftPos = ODO_LEFT_UP;

    private boolean odoButton = false;
    private boolean odoUp = true;

    @Override
    public void init() {
        odoLiftMid = hardwareMap.servo.get("odo_lift_mid");
        odoLiftLeft = hardwareMap.servo.get("odo_lift_left");
    }

    public void run(boolean odoButton) {
        if(odoButton && !this.odoButton) {
            this.odoButton = true;
            odoUp = !odoUp;
            odoMidPos = (odoUp) ? ODO_MID_UP : ODO_MID_DOWN;
            odoLeftPos = (odoUp) ? ODO_LEFT_UP : ODO_LEFT_DOWN;
        } else if(!odoButton && this.odoButton) {
            this.odoButton = false;
        }

        odoLiftMid.setPosition(odoMidPos);
        odoLiftLeft.setPosition(odoLeftPos);

        telemetry.addData("odoUp", odoUp);
    }

    public void liftOdoUp() {
        odoLiftMid.setPosition(ODO_MID_UP);
        odoLiftLeft.setPosition(ODO_LEFT_UP);
    }

    public void liftOdoDown() {
        odoLiftMid.setPosition(ODO_MID_DOWN);
        odoLiftLeft.setPosition(ODO_LEFT_DOWN);
    }
}