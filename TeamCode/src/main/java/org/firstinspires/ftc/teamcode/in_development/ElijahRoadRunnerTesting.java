package org.firstinspires.ftc.teamcode.in_development;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Roadrunner.MecanumDrive;

@Autonomous(name= "Elijah RoadRunner Testing")
@Disabled
public class ElijahRoadRunnerTesting  extends LinearOpMode{


    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        DcMotor motor = hardwareMap.dcMotor.get("arm");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Servo servo= hardwareMap.servo.get("servo");
        servo.setPosition(0);
        waitForStart();
/// moves the robot
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))

                        .lineToX(5)
                        .stopAndAdd(new ServoAction(servo, 1))
                        .stopAndAdd(new MotorAction(motor, 1))
                        .lineToX(-2)
                        .build());




    }

    ///makes the servo be able to be used
    public class ServoAction implements Action {
        Servo Servo;
        double position;
        public ServoAction(Servo s, double p) {
            this.Servo = s;
            this.position = p;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Servo.setPosition(position);
            return false;
        }
    }
    public class patientServoAction implements Action {
        Servo Servo;
        double position;
        ElapsedTime timer;

        boolean hasInitialized;

        public patientServoAction(Servo s, double p) {
            this.Servo = s;
            this.position = p;
        }


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasInitialized) {
                timer = new ElapsedTime();
                Servo.setPosition(position);
            }

            // do we need to keep running
            return timer.seconds() < 3;

        }
    }




    public class MotorAction implements Action {
        DcMotor motor;

        double power;

        ElapsedTime timer;

        boolean hasInitialized;

        public MotorAction(DcMotor m, double p) {
            this.motor = m;
            this.power = p;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasInitialized) {
                timer = new ElapsedTime();
                motor.setPower(power);
            }

            // do we need to keep running
            return timer.seconds() < 3;

        }









            }
        }


