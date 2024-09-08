package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.MainConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Angle;
import org.firstinspires.ftc.teamcode.robotcorelib.math.control.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.geometry.Vector;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.ParametricGuidingVectorField;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

import java.util.ArrayList;

@TeleOp(name = "TeleOpAuto")
public class TeleOpAuto extends OpModePipeline {
    MainConfig subsystems = new MainConfig();

    ParametricGuidingVectorField follower = new ParametricGuidingVectorField(this);

    ArrayList<Double> x = new ArrayList<Double>();
    ArrayList<Double> y = new ArrayList<Double>();
    ArrayList<Double> heading = new ArrayList<Double>();
    int posIdx = 0;

    SimplePID moveToPointPID = new SimplePID(0.03, 0.0, 0.0, 0.0, 0.5);
    private SimplePID turnPid = new SimplePID(-1.5, -0.01, 0.0, -0.8, 0.8);

    boolean pressingA = false;
    boolean pressingB = false;
    boolean pressingX = false;
    boolean pressingY = false;

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();

        subsystems.odoLift.liftOdoDown();
    }

    public void start() {
        super.start();
    }

    public void loop() {
        Robot.update();

        Pose2d robotPose = Robot.getRobotPose();

        if(gamepad1.a && !pressingA) {
            pressingA = true;

            x.add(robotPose.getX());
            y.add(robotPose.getY());
            heading.add(robotPose.getHeading());
        } else if(!gamepad1.a && pressingA) {
            pressingA = false;
        }

        if(gamepad1.b && !pressingB) {
            pressingB = true;

            posIdx += 1;
            if(posIdx >= x.size()) {
                posIdx = 0;
            }
        } else if(!gamepad1.b && pressingB) {
            pressingB = false;
        }

        if(gamepad1.y && !pressingY) {
            pressingY = true;

            PathBuilder pb = new PathBuilder();
            pb = pb.start(new Pose2d(x.get(0), y.get(0), heading.get(0)));
            for (int i = 1; i < x.size() - 1; i++) {
                pb = pb.addGuidePoint(new Pose2d(x.get(i), y.get(i), heading.get(i)));
            }
            Path p = pb.end(new Pose2d(x.get(x.size() - 1), y.get(y.size() - 1), heading.get(heading.size() - 1))).build();

            follower.followPathAsync(p);
        } else if(!gamepad1.y && pressingY) {
            pressingY = false;
        }

        if(follower.following && gamepad1.y) {
            follower.update();
        } else if(gamepad1.x && x.size() > 0) {
            double delX = x.get(posIdx) - robotPose.getX();
            double delY = robotPose.getY() - y.get(posIdx);
            Vector vel = new Vector(delX, delY);
            double distance = vel.magnitude();
            vel = vel.divide(distance);
            vel = vel.multiply(moveToPointPID.run(distance));
            double turnOutput = turnPid.run(Angle.diff(robotPose.getHeading(), heading.get(posIdx)));
            double[] motorPowers = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(vel.x, vel.y, turnOutput));
            subsystems.drivetrain.setPowers(motorPowers);
        } else {
            subsystems.drivetrain.mechanumDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }
//        subsystems.drivetrain.setPowers(DriveKinematics.mecanumFieldVelocityToWheelVelocities(
//                robotPose,
//                new Pose2d(
//                    -gamepad1.left_stick_y,
//                    gamepad1.left_stick_x,
//                    gamepad1.right_stick_x
//                )
//        ));


        subsystems.lift.run(
                gamepad2.a==true,
                gamepad2.left_trigger>0,
                gamepad2.b==true,
                gamepad2.x==true,
                gamepad2.y==true,
                gamepad2.left_bumper==true,
                gamepad2.dpad_right==true,
                gamepad2.touchpad_finger_1==true,
                gamepad2.right_bumper==true,
                gamepad2.right_trigger > 0,
                -gamepad2.left_stick_y,
                -gamepad2.right_stick_y,
                gamepad2.dpad_up==true,
                gamepad2.dpad_down==true,
                false
        );
        telemetry.update();
    }

    private double joystickCurve(double x) {
        return (x/1.07) * (0.62*Math.pow(x, 2) + 0.45);
    }

    public void stop() {
        follower.following = false;
        super.stop();
    }
}
