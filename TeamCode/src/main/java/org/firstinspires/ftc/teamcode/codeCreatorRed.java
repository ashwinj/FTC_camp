//Single driver field centric for the blur alliance

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedHashMap;

//@Disabled
@TeleOp(name = "codeCreatorRed", group = "Taus")
//@Config

public class codeCreatorRed extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = true;
    boolean isBPressed = false;
    boolean clawClosed = true;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean dpadPressed = false;
    boolean leftStick = false;
    boolean accelerating = true;
    boolean isYPressed = false;
    boolean isBlockerDown = true;
    boolean RingIn = false;
    boolean intakingRing = true;
    double rings = 0;
    double anglingPow = 0;
    boolean shooting = false;
    boolean recording = false;
    boolean isLeftStick = false;
	boolean r0 = true;
	boolean r1 = true;
	boolean r4 = true;
	boolean middleGoal = false;
    int a = 0;
	String programname = "moving"+a;

    double multiplier = 1;
    double speedFactor = 1;
    double previousY = 0;
    double previousX = 0;
    double prevMagnitude = 0;
    double calcTime;
    double stoptime = 0;
    boolean firstShot = true;
    boolean intakingInitially;
    double scale = 1;
    //double calcTime=0;
    double currentTime=0;
    double leftWheelInches;
    double rightWheelInches;
    double middleWheelInches;
    LinkedHashMap<Double, Double> speedsBl= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> speedsBr= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> speedsFr= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> speedsFl= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> speedsFlywheel= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> speedsIntake= new LinkedHashMap<Double, Double>();
    LinkedHashMap<Double, Double> toAngle= new LinkedHashMap<Double, Double>();
    ArrayList<Double> drop= new ArrayList<Double>();
    ArrayList<Double> pickUp= new ArrayList<Double>();
    ArrayList<Double> indexer= new ArrayList<Double>();
    Thread rec = new RecordingThread();
    Thread play = new PlaybackThread();
    Thread angling = new Adjusting();
    Thread pos = new Position();

    @Override
    //run file
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(method.magic8());
        telemetry.update();

        method.robot.shooter.setVelocityPIDFCoefficients(method.p,method.i,method.d,method.f);
        method.resetAngle = method.getHeading();
        while(!isStarted()) {
            telemetry.addData("system", method.robot.imu.getSystemStatus());
            telemetry.addData("status", method.robot.imu.getCalibrationStatus());
            telemetry.addLine(method.magic8());
            telemetry.update();
        }

        //File file = new File(method.captureDirectory, filename);

        waitForStart();

        //rec.start();
        method.setShooterPower(method.shooterPower);
        method.controlIndexServo(1);
        method.controlBlocker(0);
        method.controlArmServo(0);//move arm up
        method.controlClawServo(.25);//open

        if(r0){
            a = 0;
        }
        else if(r1){
            a = 1;
        }
        else if(r4){
            a = 4;
        }

        method.runtime2.reset();
        method.runtime3.reset();
        pos.start();
        while (opModeIsActive()) {
            method.robot.shooter.setVelocityPIDFCoefficients(method.p,method.i,method.d,method.f);
            drive();

            shoot();
            intake();
            claw();
            confirm();
            playBack();

            //shoot();
            //updateShootingParameters();
            toAngle();
            powerShot();
            startstop();

            //goToPosition();
            //updatePosition();
            resetAngle();
            ringIn();

            //telemetry.addData("angle", (int)method.getHeading());
            //telemetry.addData("target", (int)method.shooterRpm);
            //telemetry.addData("numRings", rings);
            //telemetry.addData("time", method.runtime3.seconds());
            if (recording) {
                telemetry.addData("recording", a);
                telemetry.addData("time", currentTime);
            }
            else{
                telemetry.addLine("not recording");
                telemetry.addData("rings", a);
                telemetry.addData("programname", programname);
            }
            telemetry.update();
            telemetry.clear();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();
            dashboardTelemetry.addData("rpm", (method.robot.shooter.getVelocity()/28.0)*60);
            dashboardTelemetry.addData("target", method.shooterRpm);
            dashboardTelemetry.addData("0", 0);
            dashboardTelemetry.update();

        }
        play.interrupt();
        rec.interrupt();
        pos.interrupt();
        angling.interrupt();

        method.setAllMotorsTo(0);
        method.setShooterPower(0);
        method.setIntakePower(0);
    }

    //drive base movement
    public void drive(){
        method.runWithEncoders();
        if (gamepad1.right_stick_button&&!leftStick){
            if (speedFactor==1){
                speedFactor = .25;
            }
            else {
                speedFactor = 1;
            }
            leftStick = true;
        }
        if(!gamepad1.right_stick_button){
            leftStick = false;
        }
        double scaleFactor = 1;
        double rotationValue = 0;
        double stickX = 0;
        double stickY = 0;

        if(Math.abs(gamepad1.right_stick_x)>.05) {
            rotationValue = gamepad1.right_stick_x;
        }
        else{
            rotationValue=0;
        }
        if(Math.abs(gamepad1.left_stick_x)>.05) {
            stickX = gamepad1.left_stick_x;
        }
        else {
            stickX=0;
        }
        if(Math.abs(gamepad1.left_stick_y)>.05) {
            stickY = -gamepad1.left_stick_y;
        }
        else {
            stickY=0;
        }
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        //Robot Centric
        //gyroAngle = Math.PI / 2;

        //inverse tangent of game-pad stick y/ game-pad stick x = angle of joystick
        double joystickAngle = Math.atan2(stickY, stickX);
        double theta =  joystickAngle+gyroAngle;

        //changing from a [+] with -- being y and | being x to an [X] with \ being y and / being x (left is forward)
        double calculationAngle = theta - ((3*Math.PI) / 4);

        //magnitude of movement using pythagorean theorem
        double magnitude = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
        double xComponent = magnitude * (Math.cos(calculationAngle));
        double yComponent = magnitude * (Math.sin(calculationAngle));

        //creates scaleFactor to make sure movement+turning doesn't exceed power 1
        if (yComponent - rotationValue > 1) {
            scaleFactor = Math.abs(yComponent - rotationValue);
        }
        if (yComponent + rotationValue > 1 && yComponent + rotationValue > scaleFactor) {
            scaleFactor = Math.abs(yComponent + rotationValue);
        }

//        if(method.runtime4.seconds()>.25) {
//            if (Math.abs(magnitude - prevMagnitude) / method.runtime4.seconds() > 1) {
//                scale = magnitude - prevMagnitude;
//                method.runtime2.reset();
//            }
//            method.runtime4.reset();
//            prevMagnitude = magnitude*multiplier;
//        }

//        multiplier = method.errorToPower(method.runtime2.seconds(), scale, 0, 1, 0);

        method.robot.frontLeftMotor.setPower((((xComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);
        method.robot.backRightMotor.setPower((((xComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);//x
        method.robot.backLeftMotor.setPower((((yComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);//y
        method.robot.frontRightMotor.setPower((((yComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);

    }

    //shooting
    public void shooter(){
        if(gamepad1.x && !isXPressed){
            isXPressed = true;
            if (!shooterOn) {
                method.setShooterPower(method.shooterPower);
                shooterOn = true;
            }
            else{
                method.setShooterPower(0);
                shooterOn = false;
            }
        }
        /*if(gamepad1.dpad_up && !dpadPressed){
            method.shooterRpm +=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_down && !dpadPressed){
            method.shooterRpm-=50;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }
        else if(gamepad1.dpad_right && !dpadPressed){
            method.shooterRpm=method.staticShooterRpm;
            method.shooterPower = (method.shooterRpm*28)/60.0;
            method.setShooterPower(method.shooterPower);
            dpadPressed=true;
        }*/

        if(!gamepad2.x){
            isXPressed = false;
        }
        //if(!gamepad1.dpad_up && !gamepad2.dpad_down && !gamepad2.dpad_right){
        //    dpadPressed = false;
        //}
    }
    public void toAngle(){
        if(gamepad2.left_bumper){
            method.currentXPosition = 84;
            method.currentYPosition = 72;
            updateShootingParameters();
            shooting = true;
            middleGoal = false;
        }
        if(gamepad2.dpad_down){
            method.currentXPosition = 108;
            method.currentYPosition = 72;
            updateShootingParameters();
            shooting = true;
            middleGoal = false;
        }
        if(gamepad2.right_bumper){
            method.currentXPosition = 132;
            method.currentYPosition = 72;
            updateShootingParameters();
            shooting = true;
            middleGoal = false;
        }
        if(gamepad2.right_trigger>.1){
            method.currentXPosition = 108;
            method.currentYPosition = 72;
            method.updateShootingParameters4();
            shooting = true;
            middleGoal = true;
        }
        if(gamepad1.left_bumper){
            updateShootingParameters();
            shooting=true;
        }

        if (shooting&&!(Math.abs(gamepad1.right_stick_x)>.1)&&!(gamepad1.right_trigger>.1)&&!(gamepad1.left_trigger>.1)){
            if(!angling.isAlive()) {
                angling.start();
                shooting = true;
            }
            //method.toAngle(method.shootingAngle, 1);
        }
        else{
            if (angling.isAlive()){
                angling.interrupt();
                anglingPow = 0;
                shooting=false;
            }
        }
    }
    public void updateShootingParameters(){
        if (middleGoal){
            method.updateShootingParameters4();
        }
        else {
            method.updateShootingParameters2();
        }

    }
    public void powerShot(){
        if(gamepad2.dpad_left){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-19, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
        }
        if(gamepad2.dpad_up){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-15, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
        }
        if(gamepad2.dpad_right){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-11, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
        }
    }

    //subsystems and utilities
    public void confirm(){
        if(gamepad1.a && !isAPressed) {
            isAPressed = true;
            if (!recording) {
                if (a == 0) {
                    if (r1) {
                        a = 1;
                    }
                    else if (r4) {
                        a = 4;
                    }
                }
                else if (a == 1) {
                    if (r4) {
                        a = 4;
                    }
                    else if (r0) {
                        a = 0;
                    }
                }
                else if (a == 4) {
                    if (r0) {
                        a = 0;
                    }
                    else if (r1) {
                        a = 1;
                    }
                }
                programname = programname.substring(0, programname.length()-1)+a;
            }
        }
        if(!gamepad1.a){
            isAPressed = false;
        }
    }
    public void intake(){
        method.robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        if (Math.abs(gamepad2.left_stick_y)>.05) {
//            //method.setIntakePower(-gamepad2.left_stick_y);
//            method.setIntakePower(-gamepad2.left_stick_y);
//            intakingRing=-gamepad2.left_stick_y>.05;
//        }
        if (Math.abs(gamepad2.left_stick_y)>.05&&rings<3||gamepad2.left_stick_button) {
            //method.setIntakePower(-gamepad2.left_stick_y);
            method.setIntakePower(-gamepad2.left_stick_y);
            intakingRing=-gamepad2.left_stick_y>.05;
        }
        else if(Math.abs(gamepad2.left_stick_y)>.05&&rings>2){
            method.setIntakePower(1*-gamepad2.left_stick_y);
        }
        else if(gamepad1.left_stick_button){
            method.setIntakePower(1);
        }
        else{
            method.setIntakePower(0);
        }
    }
    public void claw(){
        if((gamepad2.b && !isBPressed)){//||(method.runtime3.seconds()>87&&method.runtime3.seconds()<90)
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw
                isRunning = true;
                method.runtime.reset();

            }
            else{
                method.controlArmServo(1);//moving arm down
                isRunning = true;
                method.runtime.reset();
            }
        }
        if(!gamepad2.b){
            isBPressed = false;
        }
        if (isRunning){
            if (!clawClosed){
                if (method.runtime.seconds() > .5) {
                    method.controlArmServo(.25);//move arm up
                    clawClosed = true;
                    isRunning = false;
                }

            }
            else{
                if (method.runtime.seconds() > .5) {
                    method.controlClawServo(.7);//opening claw
                    clawClosed = false;
                    isRunning = false;
                }
            }
        }
    }
    public void ringIn(){
        if(!RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)<5){
            RingIn = true;
            intakingInitially = intakingRing;
        }
        if(RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)>5){
            RingIn= false;
            if(intakingRing&&intakingInitially) {
                rings++;
            }
            else if(!intakingRing&&!intakingInitially){
                rings--;
            }
        }

    }
    public void resetAngle() {
        if (gamepad1.right_bumper) {
            method.resetAngle = method.getHeading() + method.resetAngle;
            method.resetAngle2 = method.getHeading2() +method.resetAngle2;
            method.robot.encoders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            method.robot.intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void startstop(){
        if (gamepad1.left_stick_button & !isLeftStick){
            recording = !recording;
            if(recording){
                speedsFl.clear();
                speedsBl.clear();
                speedsFr.clear();
                speedsBr.clear();
                drop.clear();
                pickUp.clear();
                indexer.clear();
                speedsFlywheel.clear();
                speedsIntake.clear();
                rec.start();
            }
            else{
                rec.interrupt();
                String filename = programname;
                File file = AppUtil.getInstance().getSettingsFile(filename);
                String k = speedsBl.toString() + "\n";
                k+=speedsBr.toString() + "\n";
                k+=speedsFl.toString() + "\n";
                k+=speedsFr.toString() + "\n";
                k+=speedsIntake.toString() + "\n";
                k+=speedsFlywheel.toString() + "\n";
                k+=drop.toString() + "\n";
                k+=pickUp.toString() + "\n";
                k+=indexer.toString() + "\n";
                k+=toAngle.toString() + "\n";
                k+=stoptime + "\n";
                ReadWriteFile.writeFile(file, k);
            }
            method.runtime3.reset();
            isLeftStick = true;
        }
        else if (!gamepad1.left_stick_button){
            isLeftStick=false;
        }
    }
    public static double round(double value, int places) {
        return Math.round(value*Math.pow(10.0, places))/Math.pow(10.0, places);
    }

    public void playBack(){
        if (gamepad1.back){
            method.runtime3.reset();
            method.controlIndexServo(1);

            //pick up wobble goal
            method.controlClawServo(.25);
            method.controlArmServo(.25);
            LinkedHashMap<Double, Double> toAnglet= new LinkedHashMap<Double, Double>();
            ArrayList<Double> dropt= drop;
            ArrayList<Double> pickUpt= pickUp;
            ArrayList<Double> indexert= indexer;
            currentTime = method.runtime3.seconds();
            play.start();
            while (stoptime>currentTime){
                currentTime = method.runtime3.seconds();
                double closest = Double.MAX_VALUE;
                for (double i : speedsBl.keySet()){
                    if (Math.abs(currentTime-i)<Math.abs(currentTime-closest)){
                        closest = i;
                    }
                }
                calcTime = closest;
                telemetry.addData("playback", currentTime);
                telemetry.addData("time", currentTime);
                telemetry.update();
                if (indexert.size()>0) {
                    if (indexert.get(0) <= currentTime) {
                        method.shootRings(3);
                        indexert.remove(0);
                    }
                }
                if (pickUpt.size()>0) {
                    if (pickUpt.get(0) <= currentTime) {
                        method.pickUpClaw(0);
                        pickUpt.remove(0);
                    }
                }
                if(dropt.size()>0) {
                    if (dropt.get(0) <= currentTime) {
                        method.dropClaw();
                        dropt.remove(0);
                    }
                }
                if(toAnglet.size()>0) {
                    for (double t :toAnglet.keySet())
                        if (t <= currentTime) {
                            method.toAngle(toAnglet.get(t), 1);
                            toAnglet.remove(t);
                        }
                }

            }
            method.setAllMotorsTo(0);
            method.runtime3.reset();
            play.interrupt();

        }
    }

    //not in use
    public void shoot(){
        if(gamepad1.x) {
            indexer.add(method.runtime3.seconds());
            method.shootRings(3);
            rings=0;
        }
    }
    private class RecordingThread extends Thread {
        public RecordingThread(){
            this.setName("RecordingThread");
        }
        @Override
        public void run() {
            //telemetry.addData("notRecording", calcTime);
            //telemetry.log().add("running");
            while (!isInterrupted()) {
                currentTime = method.runtime3.seconds();
                if (recording) {
                    //telemetry.addData("recording", calcTime);
                    speedsBl.put(currentTime, method.robot.backLeftMotor.getVelocity());
                    speedsFl.put(currentTime, method.robot.frontLeftMotor.getVelocity());
                    speedsBr.put(currentTime, method.robot.backRightMotor.getVelocity());
                    speedsFr.put(currentTime, method.robot.frontRightMotor.getVelocity());
                    speedsIntake.put(currentTime, method.robot.intake.getPower());
                    speedsFlywheel.put(currentTime, method.robot.shooter.getVelocity());
                    stoptime = currentTime;
                }
            }
        }
    }
    private class PlaybackThread extends Thread {
        public PlaybackThread(){
            this.setName("PlaybackThread");
        }
        @Override
        public void run() {
            //telemetry.addData("notRecording", calcTime);
            //telemetry.log().add("running");
            while (!isInterrupted()) {
                currentTime = method.runtime3.seconds();
                //telemetry.addData("recording", calcTime);
                method.robot.backLeftMotor.setVelocity(speedsBl.get(calcTime));
                method.robot.frontRightMotor.setVelocity(speedsFr.get(calcTime));
                method.robot.frontLeftMotor.setVelocity(speedsFl.get(calcTime));
                method.robot.backRightMotor.setVelocity(speedsBr.get(calcTime));
                method.robot.intake.setPower(speedsIntake.get(calcTime));
                method.robot.intake2.setPower(-speedsIntake.get(calcTime));
                method.robot.shooter.setVelocity(speedsFlywheel.get(calcTime));
                }
            }
        }
    private class Adjusting extends Thread {
        public Adjusting(){
            this.setName("Adjusting");
        }
        @Override
        public void run() {
            while(!isInterrupted()) {
                method.updateShootingParameters4();

                anglingPow = method.toAngle2(method.shootingAngle, 1);
            }
        }
    }
    private class Position extends Thread {
        public Position() {
            this.setName("Position");
        }

        @Override
        public void run() {
            while (!isInterrupted()) {
                if (gamepad1.dpad_left) {
                    method.currentXPosition=108;
                    method.currentYPosition=72;
                }
                leftWheelInches = (method.robot.intake2.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.yMult;
                rightWheelInches = (method.robot.encoders.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.yMult;
                middleWheelInches = (method.robot.intake.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.xMult;
                double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

                double currentY = (leftWheelInches+rightWheelInches)/2;
                double deltaY1 = currentY-previousY;
                //telemetry.addData("Delta y1", deltaY1);

                double currentX = middleWheelInches-((method.getHeadingRaw()/360)*method.encCircX);
                double deltaX1 = currentX-previousX;
                //telemetry.addData("Delta x1", deltaX1);

                double thetaX = 0;
                double thetaY = Math.PI/2;

                //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
                //double rotatedTheta = theta + (Math.PI / 4);
                double gyroAngle = method.getHeading() * (Math.PI / 180); //Converts gyroAngle into radians

                double calculationAngleX =  thetaX-gyroAngle;
                double calculationAngleY =  thetaY-gyroAngle;

                double deltaY2 = Math.sin(calculationAngleY) * deltaY1;
                if(Math.abs(Math.sin(calculationAngleX) * deltaX1)>Math.abs(Math.sin(calculationAngleY) * deltaY1)){
                    deltaY2 = Math.sin(calculationAngleX) * deltaX1;
                }
                if(((Math.sin(calculationAngleY) * deltaY1)>0&&(Math.sin(calculationAngleX) * deltaX1)<0)||((Math.sin(calculationAngleY) * deltaY1)<0&&(Math.sin(calculationAngleX) * deltaX1)>0)) {
                    deltaY2 = (Math.sin(calculationAngleY) * deltaY1) + (Math.sin(calculationAngleX) * deltaX1);
                }
                double deltaX2 = Math.cos(calculationAngleX) * deltaX1;
                if(Math.abs(Math.cos(calculationAngleY) * deltaY1)>Math.abs(Math.cos(calculationAngleX) * deltaX1)){
                    deltaX2 = Math.cos(calculationAngleY) * deltaY1;
                }
                if((((Math.cos(calculationAngleY) * deltaY1)>0)&&(Math.cos(calculationAngleX) * deltaX1)<0)||(((Math.cos(calculationAngleY) * deltaY1)<0)&&((Math.cos(calculationAngleX) * deltaX1)>0))) {
                    deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX) * deltaX1;
                }

                if(true) {
                    method.currentYPosition += deltaY2;

                    if(method.currentYPosition<9){
                        method.currentYPosition = 9;
                    }

                    else if (method.currentYPosition>132.5){
                        method.currentYPosition = 132.5;
                    }

                    method.currentXPosition += deltaX2;

                    if(method.currentXPosition<9){
                        method.currentXPosition = 9;
                    }

                    else if (method.currentXPosition>132.5){
                        method.currentXPosition = 132.5;
                    }
                    previousX = currentX;
                    previousY = currentY;
//                    telemetry.addData("position", "[" +(int)method.currentXPosition + ", " + (int)method.currentYPosition + "]");
//                    telemetry.addData("calcAnglex", calculationAngleX*(180/Math.PI));
//                    telemetry.addData("calcAngley", calculationAngleY*(180/Math.PI));
//                    telemetry.addData("current X", currentX);
//                    telemetry.addData("current Y", currentY);
//                    telemetry.update();
                }
                toAngle();
            }
        }
    }
    }
