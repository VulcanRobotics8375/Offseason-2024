package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.EmptyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp
public class MotorTest extends OpModePipeline {
    DcMotorEx lift;
    private boolean switchButton = false;
    private boolean setPower = true;
    private int holdPosition = 0;

    public void init(){
        runMode = RobotRunMode.TELEOP;
        subsystems = new EmptyConfig();
        super.init();
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
        Robot.update();

        if(gamepad1.a && !switchButton) {
            switchButton = true;
            setPower = !setPower;
        } else if(!gamepad1.a && switchButton) {
            switchButton = false;
        }

        if(setPower) {
            if(lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                lift.setPower(0.0);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                lift.setPower(-gamepad1.right_stick_y * 0.3);
            }
            telemetry.addData("STATE", "setPower");
        } else {
            holdPosition = Range.clip(holdPosition + (int)(gamepad1.left_stick_y*5), 0, 850);

            lift.setTargetPosition(holdPosition);
            if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            lift.setPower(1.0);
            telemetry.addData("STATE", "runToPosition");
        }

        telemetry.addData("pos", lift.getCurrentPosition());
        telemetry.addData("current", lift.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("power", lift.getPower());

        telemetry.update();
    }
}
