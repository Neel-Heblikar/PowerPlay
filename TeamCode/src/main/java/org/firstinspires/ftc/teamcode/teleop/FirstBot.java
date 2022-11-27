package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.*;

@TeleOp(name = "PowerPlay First Bot", group = "TestBot")
public class FirstBot extends LinearOpMode {
    SampleMecanumDrive drive;

    DcMotor liftL = null;
    DcMotor liftR = null;
    DcMotor liftT = null;

    Servo clawL = null;
    Servo clawR = null;
    Servo extend = null;

    DcMotor turretR = null;

    Gamepad g1 = gamepad1;
    Gamepad g2 = gamepad2;

    double extensionPos = Constants.extendInPos;
    double extensionRange = Constants.extendOutPos - Constants.extendInPos;
    boolean isHolding = false;

    int holdPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        g1 = gamepad1; //Defining Variables for the gamepad - Neel 
        g2 = gamepad2; 

        Constants.initHardware(hardwareMap);

        liftL = Constants.liftL;
        liftR = Constants.liftR;
        liftT = Constants.liftT;

        clawL = Constants.clawL; //Initializing left and right servo motors - Neel Heblikar
        clawR = Constants.clawR;

        extend = Constants.extend;

        turretR = Constants.turretR;

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            teleopCode();
            teleopAuto();
        }
    }

    public void teleopAuto() {
        // when "y" key is held, lift will move up until it hits the target - Neel Heblikar
        if ((g2.dpad_up || g2.y) && Math.abs(liftR.getCurrentPosition() - Constants.liftTargetHigh) >= Constants.liftError) {
            Constants.setLift(Constants.liftTargetHigh, Constants.liftPower); // you can see this method in Constants
        }
        // when "a" key is held, lift will move down until it reaches 0 - Neel Heblikar
        if ((g2.dpad_down || g2.a) && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError) {
            Constants.setLift(0, Constants.liftPower);
        }
        //When d is held, lift will stay up - Neel Heblikar

        // if y and a are not present, right joystick will control - Neel Heblikar
        if (!g2.dpad_down && !g2.dpad_up && !g2.a && !g2.y && !g2.dpad_left) {
            if (g2.left_stick_y == 0 && Math.abs(liftR.getCurrentPosition()) >= Constants.liftError){
                if (!isHolding){
                    isHolding = true;
                    holdPos = liftR.getCurrentPosition();
                }
                Constants.setLift(holdPos, Constants.liftPower);
            }
            else {
                liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // squared input - Neel Heblikar
                double liftInput = Math.pow(gamepad2.left_stick_y, 2);
                if (gamepad2.left_stick_y < 0) liftInput *= -1;

                isHolding = false;
                holdPos = 0;

                // Opposite powers - Neel Heblikar
                if (liftR.getCurrentPosition() > -4500 || gamepad2.left_stick_y < 0) {
                    liftL.setPower(-liftInput);
                    liftR.setPower(liftInput);
                    liftT.setPower(-liftInput);
                }
            }
        }

        // if Dpad is right, the turret will move right 90 degrees - Neel Heblikar
        if (g2.b && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget90) >= Constants.turretError) {
            Constants.setTurret(90, false, Constants.turretPower); // look at this method in Constants
        }
        //if Dpad is left, turret will move 90 degrees left - Neel Heblikar
        else if (g2.x && Math.abs(turretR.getCurrentPosition() - Constants.turretTargetNeg90) >= Constants.turretError) {
            Constants.setTurret(-90, false, Constants.turretPower);
        }
        //if Dpad is down, turret will move back 180 degrees - Neel Heblikar
        else if (g2.y && Math.abs(turretR.getCurrentPosition() - Constants.turretTarget180) >= Constants.turretError) {
            Constants.setTurret(180, false, Constants.turretPower);
        }
        //if Dpad is up, turret goes to 0 degrees - Neel Heblikar
        else if (g2.a && Math.abs(turretR.getCurrentPosition()) >= Constants.turretError) {
            Constants.setTurret(0, false, Constants.turretPower);
        }
        //If no dpads are pressed, left joystick will control turret
        else if (!g2.x && !g2.a && !g2.y && !g2.b) {
            turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // squared input, 0.5 speed - Neel Heblikar
            double turretInput = -Math.pow(gamepad2.right_stick_x, 2) * 0.55;
            // Slow Mode = Left Trigger - Neel Heblikar
//            if (g2.left_trigger == 1) turretInput *= 0.7;
            if (gamepad2.right_stick_x < 0) turretInput *= -1;

            turretR.setPower(turretInput);
        }

        // if "b" key is pressed, it resets all encoders - Neel Heblikar
        if (g2.dpad_right) {
            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            liftT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void teleopCode() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // defining default Speed - Neel Heblikar
        double multiplier = 0.75;
        // curve to apply (squared rn)
        double power = 2.0;

        // slow mode = right trigger - Neel Heblikar
        if (g1.right_bumper) {
            multiplier = 0.3;
        }
        // fast mode = left trigger - Neel Heblikar
        if (g1.left_bumper) {
            multiplier = 1;
        }

        double yMult = multiplier;
        double xMult = multiplier;
        double turnMult = multiplier;

        if (y < 0) yMult *= -1;
        if (x < 0) xMult *= -1;
        if (turn < 0) turnMult *= -1;

        y = Math.pow(y, power) * yMult;
        x = Math.pow(x, power) * xMult;
        turn = Math.pow(turn, power) * turnMult;

        drive.setWeightedDrivePower(new Pose2d(y, x, turn));

        // Close Claw = right bumper - Neel Heblikar
        if (gamepad2.right_bumper) {
            Constants.setClaw(Constants.ClawPosition.CLOSED);
        }
        // Close Claw = left bumper - Neel Heblikar
        if (gamepad2.left_bumper) {
            Constants.setClaw(Constants.ClawPosition.OPEN);
        }

        // extension goes outward if right trigger is held - Neel Heblikar
        // Constants.extendInPos = Trigger is at 0 - Neel Heblikar
        // Constants.extendOutPos = Trigger is at 1 - Neel Heblikar
        // Between = set proportional - Neel Heblikar

//        double extensionValue = (Math.sqrt(gamepad2.right_trigger) + Math.sqrt(gamepad2.left_trigger)) / 2.0; - Neel Heblikar
//        extend.setPosition((1 - extensionValue) * (Constants.extendInPos - Constants.extendOutPos) + Constants.extendOutPos); - Neel Heblikar

        if (gamepad2.right_trigger <= 1)
            extensionPos += (extensionRange * gamepad2.right_trigger * 0.03);
//        else if (gamepad2.right_trigger == 1)
//            extensionPos = Constants.extendOutPos;

        if (gamepad2.left_trigger < 1)
            extensionPos -= (extensionRange * gamepad2.left_trigger * 0.03);
        else if (gamepad2.left_trigger == 1)
            extensionPos = Constants.extendInPos;

        if (extensionPos < Constants.extendOutPos)
            extensionPos = Constants.extendOutPos;
        if (extensionPos > Constants.extendInPos)
            extensionPos = Constants.extendInPos;

        if (g2.y || g2.a)
            extensionPos = Constants.extendInPos;

        extend.setPosition(extensionPos);


        telemetry.addData("Turret power", turretR.getPower());
        telemetry.addData("Turret position", turretR.getCurrentPosition());
        telemetry.addData("Turret target", turretR.getTargetPosition());

        telemetry.addData("Lift position", liftR.getCurrentPosition());
        telemetry.addData("Lift power", liftL.getPower());
        telemetry.addData("LiftT target", liftT.getTargetPosition());

        telemetry.addData("Extension position", extend.getPosition());

        telemetry.update();
    }
}
