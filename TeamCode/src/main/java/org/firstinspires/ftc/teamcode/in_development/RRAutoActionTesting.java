package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Autonomous(name="RR Auto Action Testing")
@Disabled
public class RRAutoActionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Servo arm = hardwareMap.servo.get("arm");
        DcMotor lift = hardwareMap.dcMotor.get("lift");

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(64)
                .stopAndAdd(new PatientServoAction(arm, 1))
                .lineToX(0)
                .stopAndAdd(new MotorRunToPositionAction(lift, 100, 1000))
                .lineToX(40)
                .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                .lineToX(64)
                .stopAndAdd(new MotorRunToPositionAction(lift, 10, 1000))
                .stopAndAdd(new WaitUntilMotorDoneAction(lift))
                .build());
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
        DcMotor motor;
        int position;
        int motorVelocity;

        public MotorRunToPositionAction(DcMotor m, int position, int motorVelocity) {
            this.motor = m;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.5);
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
