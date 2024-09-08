//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.robot.LiftConfig;
//import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
//import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
//import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
//
//@TeleOp(name = "LiftOpMode")
//public class LiftOpMode extends OpModePipeline {
//    LiftConfig subsystems = new LiftConfig();
//
//    public void init() {
//        super.subsystems = subsystems;
//        runMode = RobotRunMode.TELEOP;
//        super.init();
//    }
//
//    public void start() {
//        super.start();
//    }
//
//    public void loop() {
//        Robot.update();
//
//        subsystems.lift.run(
//                gamepad1.y,
//                gamepad1.x,
//                gamepad1.a,
//                gamepad1.b
//
//        );
//
//        telemetry.update();
//    }
//
//
//
//}
