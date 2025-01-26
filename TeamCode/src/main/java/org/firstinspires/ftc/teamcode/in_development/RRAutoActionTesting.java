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

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name="RR Auto Action Testing")
@Disabled
public class RRAutoActionTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        Servo intakeLeft = hardwareMap.servo.get("servo");
        DcMotor motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(new Pose2d(0,0,0))
                .lineToX(64)
                .stopAndAdd(new PatientServoAction(intakeLeft, 1))
                .lineToX(0)
                .stopAndAdd(new PatientMotorAction(motor, 100))
                .lineToX(40)
                .stopAndAdd(new PatientMotorAction(motor, 10))
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

    public class PatientMotorAction implements Action {
        DcMotor motor;
        int position;

        public PatientMotorAction(DcMotor m, int pos) {
            this.motor = m;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            motor.setTargetPosition(position);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(.5);
            return motor.isBusy();
        }
    }
}
