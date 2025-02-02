package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="RR Auto Action Testing")
@Disabled
public class RRAutoActionTesting extends LinearOpMode {

    // Constants
    private static final double LIFT_TICKS_PER_MM = 28 * 12 / 120.0; // RevRobotics 28 ticks/rev motor, with 12:1 gear reduction, and belt travel of 120mm/rev
    private static final int LIFT_VELOCITY = 2100;
    private static final int LIFT_COLLAPSED_INTO_ROBOT = 0;
    private static final double LIFT_SPECIMEN_ABOVE_SIDEWALL = 50 * LIFT_TICKS_PER_MM; // TODO: Example value, replace with actual value
    private static final double LIFT_SCORE_SPECIMEN = 100 * LIFT_TICKS_PER_MM; // TODO: Example value, replace with actual value
    private static final double INTAKE_COLLECT = -1.0;
    private static final double INTAKE_OFF = 0.0;
    private static final double INTAKE_DEPOSIT = 0.5;
    private static final int SLIDE_FULLY_EXTENDED = 300; // TODO: Example value, replace with actual value
    private static final double SLIDE_TICKS_PER_MM = 28 * 12 / 120.0; // RevRobotics 28 ticks/rev motor, with 12:1 gear reduction, and belt travel of 120mm/rev
    private static final int SLIDE_COLLAPSED_INTO_ROBOT = 0;
    private static final int SLIDE_VELOCITY = 2100;
    private static final double ARM_SCORE_SPECIMEN = .5; // TODO: Example value, replace with actual value
    private static final double ARM_GRAB_SPECIMEN = -1; // TODO: Example value, replace with actual value
    private static final double ARM_TRANSFER = 1; // TODO: Example value, replace with actual value
    private static final double CLAW_OPEN = 1; // TODO: Example value, replace with actual value
    private static final double CLAW_CLOSED = 0; // TODO: Example value, replace with actual value

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Servo arm = hardwareMap.servo.get("arm");
        Servo claw = hardwareMap.servo.get("claw");
        //CRServo intake = hardwareMap.crservo.get("intake");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor leftActuator = hardwareMap.dcMotor.get("left_actuator");
        DcMotor rightActuator = hardwareMap.dcMotor.get("right_actuator");

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(-8, 61.7, Math.toRadians(-90)))
                    // start - swing arm to score specimen position and move toward high rung
                    .stopAndAdd(new ServoAction(arm, ARM_SCORE_SPECIMEN))
                    .setTangent(Math.toRadians(-90))
                    .lineToY(35)
                    // raise lift until specimen has been scored, then let go of specimen, rotate arm back and lower lift
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SCORE_SPECIMEN, LIFT_VELOCITY))
                    .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                    .stopAndAdd(new ServoAction(arm, ARM_GRAB_SPECIMEN))
                    .stopAndAdd(new MotorRunToPositionAction(lift, LIFT_COLLAPSED_INTO_ROBOT, LIFT_VELOCITY))

                    // Move around submersible to push samples to observation zone
                    .setTangent(Math.toRadians(0))
                    .lineToX(-16)
                    .splineToLinearHeading(new Pose2d(-35,32,Math.toRadians(-90)), Math.toRadians(-90))
                    .lineToY(20)
                    .splineToLinearHeading(new Pose2d(-45,12,Math.toRadians(-90)), Math.toRadians(90))
                    // Push sample 1 to observation zone
                    .lineToY(53)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(-53,10,Math.toRadians(-90)), Math.toRadians(90))
                    // Push sample 2 to observation zone
                    .lineToY(53)
                    .lineToY(12)
                    .splineToLinearHeading(new Pose2d(-61,10,Math.toRadians(-90)), Math.toRadians(90))
                    // Push sample 3 to observation zone
                    .lineToY(53)

                    // Grab specimen 2 from sidewall
                    .splineToConstantHeading(new Vector2d(-36,61.7), Math.toRadians(90))
                    .stopAndAdd(new ServoAction(claw, CLAW_CLOSED))
                    .waitSeconds(.5)
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SPECIMEN_ABOVE_SIDEWALL, LIFT_VELOCITY))
                    .stopAndAdd(new ServoAction(arm, ARM_SCORE_SPECIMEN))
                    // Move to high rung
                    .splineToConstantHeading(new Vector2d(0,35), Math.toRadians(-90))
                    // raise lift until specimen has been scored, then let go of specimen, rotate arm back and lower lift
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SCORE_SPECIMEN, LIFT_VELOCITY))
                    .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                    .stopAndAdd(new ServoAction(arm, ARM_GRAB_SPECIMEN))
                    .stopAndAdd(new MotorRunToPositionAction(lift, LIFT_COLLAPSED_INTO_ROBOT, LIFT_VELOCITY))

                    // Grab specimen 3 from sidewall
                    .setTangent(Math.toRadians(140))
                    .splineToConstantHeading(new Vector2d(-36,61.7), Math.toRadians(90))
                    .stopAndAdd(new ServoAction(claw, CLAW_CLOSED))
                    .waitSeconds(.5)
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SPECIMEN_ABOVE_SIDEWALL, LIFT_VELOCITY))
                    .stopAndAdd(new ServoAction(arm, ARM_SCORE_SPECIMEN))
                    // Move to high rung
                    .splineToConstantHeading(new Vector2d(0,35), Math.toRadians(-90))
                    // raise lift until specimen has been scored, then let go of specimen, rotate arm back and lower lift
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SCORE_SPECIMEN, LIFT_VELOCITY))
                    .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                    .stopAndAdd(new ServoAction(arm, ARM_GRAB_SPECIMEN))
                    .stopAndAdd(new MotorRunToPositionAction(lift, LIFT_COLLAPSED_INTO_ROBOT, LIFT_VELOCITY))

                    // Grab specimen 4 from sidewall
                    .setTangent(Math.toRadians(140))
                    .splineToConstantHeading(new Vector2d(-36,61.7), Math.toRadians(90))
                    .stopAndAdd(new ServoAction(claw, CLAW_CLOSED))
                    .waitSeconds(.5)
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SPECIMEN_ABOVE_SIDEWALL, LIFT_VELOCITY))
                    .stopAndAdd(new ServoAction(arm, ARM_SCORE_SPECIMEN))
                    // Move to high rung
                    .splineToConstantHeading(new Vector2d(0,35), Math.toRadians(-90))
                    // raise lift until specimen has been scored, then let go of specimen, rotate arm back and lower lift
                    .stopAndAdd(new MotorRunToPositionAction(lift, (int) LIFT_SCORE_SPECIMEN, LIFT_VELOCITY))
                    .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                    .stopAndAdd(new ServoAction(arm, ARM_GRAB_SPECIMEN))
                    .stopAndAdd(new MotorRunToPositionAction(lift, LIFT_COLLAPSED_INTO_ROBOT, LIFT_VELOCITY))

                    // Move to observation zone
                    .setTangent(Math.toRadians(140))
                    .splineToConstantHeading(new Vector2d(-36,61.7), Math.toRadians(90))

                    .build());
    }

    public class CRServoAction implements Action {
        CRServo crservo;
        double power;

        public CRServoAction(CRServo s, double pow) {
            this.crservo = s;
            this.power = pow;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            crservo.setPower(power);
            return false;
        }
    }

    public class ServoAction implements Action {
        Servo servo;
        double position;

        public ServoAction(Servo s, double pos) {
            this.servo = s;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(position);
            return false;
        }
    }

    public class PatientServoAction implements Action {
        Servo servo;
        double position;
        ElapsedTime timer;

        public PatientServoAction(Servo s, double pos) {
            this.servo = s;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null){
                timer = new ElapsedTime();
                servo.setPosition(position);
            }

            // do we need to keep running?
            return timer.seconds() < 3;
        }
    }

    public class MotorRunToPositionAction implements Action {
        DcMotorEx motorEx;
        int position;
        int motorVelocity;

        public MotorRunToPositionAction(DcMotor m, int position, int motorVelocity) {
            this.motorEx = (DcMotorEx) m;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motorEx.setTargetPosition(position);
            motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorEx.setVelocity(motorVelocity);
            return false;
        }
    }

    public class WaitUntilMotorDoneAction implements Action {
        DcMotor motor;

        public WaitUntilMotorDoneAction(DcMotor m) {
            this.motor = m;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !motor.isBusy();
        }
    }

}
