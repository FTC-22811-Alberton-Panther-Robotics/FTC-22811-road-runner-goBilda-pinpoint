package org.firstinspires.ftc.teamcode.in_development;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

/**
 *   This is an example of an autonomous mode that uses the Road Runner library.
 *   Here is a link to the website where each of these steps are spelled out: https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
 *   I have adapted it to refer to two lift motors and renamed "claw" to "gripper"
 */
@Config
@Autonomous(name = "RR Centerstage Auto Example", group = "Autonomous")
@Disabled
public class RRCenterstageAutoExample extends LinearOpMode {
    public class Lift {
        private DcMotorEx leftLift;
        private DcMotorEx rightLift;

        public Lift(HardwareMap hardwareMap) {
            leftLift = hardwareMap.get(DcMotorEx.class, "left_lift");
            rightLift = hardwareMap.get(DcMotorEx.class, "right_lift");
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLift.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftLift.setPower(0.8);
                    rightLift.setPower(0.8);
                    initialized = true;
                }

                double pos = leftLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftLift.setPower(-0.8);
                    rightLift.setPower(-0.8);
                    initialized = true;
                }

                double pos = leftLift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    leftLift.setPower(0);
                    rightLift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Gripper {
        private Servo gripper;

        public Gripper(HardwareMap hardwareMap) {
            gripper = hardwareMap.get(Servo.class, "gripper");
        }

        public class CloseGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(0.55);
                return false;
            }
        }
        public Action closeGripper() {
            return new CloseGripper();
        }

        public class OpenGripper implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gripper.setPosition(1.0);
                return false;
            }
        }
        public Action openGripper() {
            return new OpenGripper();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Gripper gripper = new Gripper(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // vision here that outputs power
        int visionOutputPosition = 1;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a gripper tightening.
        Actions.runBlocking(gripper.closeGripper());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        gripper.openGripper(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}