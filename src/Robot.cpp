#include "WPILib.h"

// FINAL ROBOT 1 CODE
// Sam Bernstein && Leo Adberg 2015 FRC
// "This game is garbage, and our robot sucks."
class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;

	bool orientEnabled = false;
	bool lastOrientButton = false;

	Compressor *c = new Compressor(0);
	Solenoid *suctionCups = new Solenoid(0);
	bool suctionCupsOn = false;
	bool lastSuctionButton = false;

	DoubleSolenoid *piston1 = new DoubleSolenoid(1,2);
	bool pistonsOn = false;
	bool lastPistonButton = false;

	Joystick *driveStick = new Joystick(0); // only joyauxStick
	Joystick *auxStick = new Joystick(1);

	DigitalInput *topLimit_L = new DigitalInput(5); // for stopping elevator at top
	DigitalInput *bottomLimit_L = new DigitalInput(2); // for stopping elevator at bottom

	Ultrasonic *ultrasonic_L = new Ultrasonic(6, 7); // CHANGE PORTS
	Ultrasonic *ultrasonic_R = new Ultrasonic(8, 9); // CHANGE PORTS

	//ROBOT 1
	Encoder *liftEncoder_L = new Encoder(0, 1, true);
	Encoder *liftEncoder_R = new Encoder(3, 4, true);
	/*
	//ROBOT 2
	Encoder *liftEncoder_L = new Encoder(10, 11, true);
	Encoder *liftEncoder_R = new Encoder(12, 13, true);
*/
	/* ?
	Encoder *leftFrontEncoder = new Encoder(5, 6, true);
	Encoder *rightFrontEncoder = new Encoder(7, 8, true);
	*/
	Encoder *leftFrontEncoder = new Encoder(10, 11, true);
	Encoder *rightFrontEncoder = new Encoder(14, 15, true);

	Encoder *leftBackEncoder = new Encoder(12, 13, true);
	Encoder *rightBackEncoder = new Encoder(16, 17, true);

	CANTalon *leftFront = new CANTalon(1);
	CANTalon *leftBack = new CANTalon(5);
	CANTalon *rightFront = new CANTalon(2);
	CANTalon *rightBack = new CANTalon(6);

	TalonSRX *PWMlf = new TalonSRX(0);
	TalonSRX *PWMlb = new TalonSRX(1);
	TalonSRX *PWMrf = new TalonSRX(2);
	TalonSRX *PWMrb = new TalonSRX(3);

	TalonSRX *testAuto = new TalonSRX(7);

	CANTalon *lift_R = new CANTalon(4);
	CANTalon *lift_L = new CANTalon(3);

	RobotDrive *drive = new RobotDrive(PWMlf,PWMlb,PWMrf,PWMrb);
	Gyro *gyro = new Gyro(0);
	Gyro *eGyro = new Gyro(1);

	Timer *autonTimer;
	Timer *suctionTimer; // we're not using this right now, but we might in the future for whatever reason

	const float cycleWaitTime = .0; // the time it waits between each cycle, in seconds (WE'RE NOT USING THIS)

	// Smooth Start elevator variables
	const int toteHeight = 517; // number of encoder ticks per 1 encoder height
	const int stopBuffer = 100; // encoder ticks away from stopping point that smoothStop starts smoothness
	const int maxHeight = 2650; // encoder ticks from very bottom to very top. This is being worked out.
	const int stopMargin = 200; // encoder ticks distance away from top for stopping (it's a safety buffer)
	const int stopHeight = maxHeight - stopMargin; // sets the stop height

	const float maxLiftSpeed = 0.85;
	const float startSpeed = .2*maxLiftSpeed; // acceleration starts here, skips the slowest part of Smooth Start
	const float startTime = .3; // How long it takes Smooth Start to reach maxLiftSpeed, in seconds
	const float increaseSpeed = (maxLiftSpeed - startSpeed)/(startTime / 0.02);
	float smoothStart = startSpeed;  // motor value during smooth start

	const float hardCorrection = 1.17;
	const float correction = 0.15;
	float correctionDifference = correction*smoothStart;
	int l_LiftEncoder;
	int r_LiftEncoder;

	int r_frontEncoder;
	int l_frontEncoder;
	int r_backEncoder;
	int l_backEncoder;

	float driveInchesPerTick = .4;

	int intermediateGyro;
	float gyroValue;
	float eGyroValue;
	int fakeZero_L = 50;
	int fakeZero_R = fakeZero_L;

	// Smooth Start AND Smooth Stop 90-Align variables
	const float northDegrees = 0.0; // number of encoder ticks per 1 encoder height
	const float eastDegrees = 90.0; // encoder ticks away from stopping point that smoothStop starts smoothness
	const float southDegrees = 180.0; // encoder ticks from very bottom to very top. This is exact.
	const float westDegrees = 270.0; // encoder ticks distance away from top for stopping (it's a safety buffer)
	const float maxDegrees = 4.0*eastDegrees;

	const float maxAlignSpeed = .8; // don't name ANYTHING this anywhere else or bad stuff will probably happen
	const float startAlignSpeed = .2*maxAlignSpeed; // acceleration starts here, skips the slowest part of Smooth Start
	const float startAlignTime = .5; // How long it takes Smooth Start to reach maxAlignSpeed, in seconds
	const float increaseAlignSpeed = (maxAlignSpeed - startAlignSpeed)/(startAlignTime / 0.02);
	float smoothAlign = startAlignSpeed;  // motor value multiplication during smooth align

	float dir = 1.0; // for reversing turning direction
	float alignBufferZone = 12.0;
	float minAlignMultiplier = 0.1;

	//driveStick buttons
	const int westButton = 1;
	const int southButton = 2;
	const int eastButton = 3;
	const int northButton = 4;
	const int getToteButton = 6;
	const int relToteButton = 8;
	const float acqStart_L = 90.0; // right button
	const float acqStart_R = 270.0; // left button
	//const int killSwitch = 9;
	const int killSwitch = 5;
	const int resetGyroButton = 10; // reset Gyro
	const int orientToggleButton = 9;
	// joysticks are for mecanum Orient Drive

	//auxStick buttons
	const int upButton = 6; // for elevator
	const int downButton = 8; // for elevator
	const int pos1Button = 1; // for elevator
	const int pos2Button = 2; // for elevator
	const int pos3Button = 3; // for elevator
	const int pos4Button = 4; // for elevator
	const int resetElevatorButton = 9;
	const int resetEncodersButton = 10; // resets elevator motor encoders
	const float slipCorrectButton = 0.0;
	const int suctionCupsButton = 5;
	const int pistonButton = 7;

	const float turnSpeed = .7;


	const float speedM = .8; // default testing drivetrain max speed
	float resetDifference = 0.0;

	int autoNumber = 0; // this tells the program which AUTO program to run
	// for stepping through the steps for each subsystem in autonomous
	int stepDrive = 0;
	int stepLift = 0;
	int stepPneum = 0;

	// function global variables:

	//ResetElevator and SlipCorrect:
	float dirxtion = 1.0; // change this if backwards with respect to right/left

	float bufferZ = 20.0;
	float defP = .2;

	float fix;
	//SlipCorrect: automatically re-calibrate lift encoders with eGyro
	bool slipCorrectRunning = false;
	int stepSlip = 0;

	float encoSlipThresh = 13.0;
	float gyroSlipThresh = 10.0;
	float gyroAlignThresh = 2.5;

	bool leftSide = 0;

	float slipAlignSpeed = .4;

	//OutputStraightDrive: go forward and backward straight with encoders
	float speedStrDrive = .9;

	float leftFrontEncStraight = 0.0;
	float rightFrontEncStraight = 0.0;
	float leftBackEncStraight = 0.0;
	float rightBackEncStraight = 0.0;

	float strLeftEnc = 0.0;
	float strRightEnc = 0.0;
	float strCorrSign = 1.0;
	float strDifference = 0.0;

	// SetDriveEncAuto: sets drive train encoder values before AUTO functions
	float avgDriveEnc = 0.0;

	bool firstCall = true;
	//
	Timer *toteTimer = new Timer();
	const float toteExtendTime = 1.7;
	float getToteLastTime = 0.0;
	// ReleaseTote: extend, unsuck, and retract

	bool relToteRunning = false;
	int stepRelTote = 0;

	// AcqGetTote:  grab, suck and bring in tote
	bool getToteRunning = false;
	int stepGetTote = 0;

	const float getToteSuckTime = 0.3;

	// AcqRoutine: acquisition routine
	int stepAcq = 0;
	int acqRunning = 0; // 0 is not running, -1 is starting from right, 1 is starting from left
	float ultrasonic_L_dist = 0.0;
	float ultrasonic_R_dist = 0.0;

	float r_FrontEncoder_1 = 0.0;
	float l_FrontEncoder_1 = 0.0;
	float r_BackEncoder_1 = 0.0;
	float l_BackEncoder_1 = 0.0;

	float r_FrontDelta = 0.0;
	float l_FrontDelta = 0.0;
	float r_BackDelta = 0.0;
	float l_BackDelta = 0.0;

	float avgStrafeTicks = 0.0;

	const float closerTote_dist = 10.0; // inches difference for recognizing with ultrasonic that robot has reached edge of tote
	const float acqStrafeSpeed = 0.9;

	float levelAlign_dist[2] = {4.0, 4.0}; // left, right

	float levelAlign_dist_L = 4.0; // inches to the side robot must strafe to line up with tote levels
	float levelAlign_dist_R = levelAlign_dist_L;
	float dirDepth = 1.0; // for storing whether to mover forward or backward
	const float toteDepth_dist = 24.0; // inches, for optimal distance away from tote for acquisition (depth)
	const float toteDepth_range = 2.5; // inches, tolerance for getting within range of tote


	int AcqIndex() {
		return (int)(.5*(acqRunning + 1)); // for conversion of {-1, 1} to {0, 1} indices
	}

	void ResetDriveEncoders() {
		leftFrontEncoder->Reset();
		rightFrontEncoder->Reset();
		leftBackEncoder->Reset();
		rightBackEncoder->Reset();
	}

	void SetDriveEncAuto() {
		if (firstCall) {
			leftFrontEncStraight = abs(leftFrontEncoder->Get());
			rightFrontEncStraight = abs(rightFrontEncoder->Get());
			leftBackEncStraight = abs(leftBackEncoder->Get());
			rightBackEncStraight = abs(rightBackEncoder->Get());

			firstCall = false;
		}

		strLeftEnc = .5*(abs(leftFrontEncoder->Get() - leftFrontEncStraight) + abs(leftBackEncoder->Get() - leftBackEncStraight));
		strRightEnc = .5*(abs(rightFrontEncoder->Get() - rightFrontEncStraight) + abs(rightBackEncoder->Get() - rightBackEncStraight));

		avgDriveEnc = 0.5*abs(strLeftEnc + strRightEnc);
	}

	void OutputLift(float reverse, float difference = 0) {
		lift_L->Set(reverse*(smoothStart + difference)*hardCorrection);
		lift_R->Set(-reverse*(smoothStart - difference));
	}
	void OutputLiftRegular(float reverse, float liftSpeed) {
		lift_L->Set(reverse*liftSpeed);
		lift_R->Set(-reverse*liftSpeed);
	}

	void OutputAllDrive(float velocity = .5) {
		PWMlf->Set(velocity);
		PWMlb->Set(velocity);
		PWMlf->Set(velocity);
		PWMlb->Set(velocity);
	}

	void OutputPointTurn(float direction, float speedTurn = .4) { // for point turning and 90-align
		PWMrf->Set(direction*speedTurn);
		PWMrb->Set(direction*speedTurn);
		PWMlf->Set(direction*speedTurn);
		PWMlb->Set(direction*speedTurn);
	}

	void OutputStraightDrive(float direction, float speed = .8) { // moves straight backward or forward indefinitely
		SetDriveEncAuto();

		strCorrSign = (strRightEnc - strLeftEnc)/abs(strRightEnc - strLeftEnc); // store sign

		strDifference = (float)(std::min(abs(strRightEnc - strLeftEnc)/100.0, 0.4)) * strCorrSign;

		PWMlf->Set(direction*(speed + strDifference));
		PWMlb->Set(direction*(speed + strDifference));

		PWMrf->Set(direction*(speed - strDifference));
		PWMrb->Set(direction*(speed - strDifference));

	}

	void OutputDistanceDrive(float direction, float distance = 1.0) { // for encoder AUTO go straight forward/backward movements
		SetDriveEncAuto();

		strCorrSign = (strRightEnc - strLeftEnc)/abs(strRightEnc - strLeftEnc); // store sign
		strDifference = (float)(std::min(abs(strRightEnc - strLeftEnc)/100.0, 0.3) * strCorrSign);

		if (avgDriveEnc*driveInchesPerTick < distance) {
			PWMlf->Set(direction*(speedStrDrive + strDifference));
			PWMlb->Set(direction*(speedStrDrive + strDifference));

			PWMrf->Set(direction*(speedStrDrive - strDifference));
			PWMrb->Set(direction*(speedStrDrive - strDifference));
		}
		else {
			firstCall = true;
			stepDrive++;
		}
	}

	void OutputStrafe(float direction, float speedStrafe = .4) { // for point turning and 90-align
		PWMrf->Set(direction*speedStrafe);
		PWMlb->Set(direction*speedStrafe);

		PWMrb->Set(-direction*speedStrafe);
		PWMlf->Set(-direction*speedStrafe);
	}

	void ResetElevator() {

		resetDifference = correction*std::min((float)abs(l_LiftEncoder-r_LiftEncoder)/50.0 + 0.5,1.0)*((l_LiftEncoder-r_LiftEncoder)/abs(l_LiftEncoder-r_LiftEncoder));

		smoothStart = 0.4;

		if (eGyroValue >= 0.0 && eGyroValue <= 80.0) {
			fix = -dirxtion*std::min(abs(eGyroValue)/bufferZ, defP);
		}
		else {
			fix = dirxtion*std::min(abs(eGyroValue - 360.0)/bufferZ, defP);
		}

		if (bottomLimit_L->Get()) {
			OutputLift(-1.0, resetDifference);
			//OutputLiftRegular(-1.0, 0.7);
		}
		else {
			liftEncoder_L->Reset();
			liftEncoder_R->Reset();
			OutputLift(0.0, 0.0);
		}
	}

	void SlipCorrect() {

		smoothStart = 0.5;

		//OutputLiftRegular(-1.0, 0.7);
		switch(stepSlip)
		{
		case 0:
			getToteLastTime = 0;
			toteTimer->Reset();
			toteTimer->Start();

			if (eGyroValue > 0.0 && eGyroValue < 80.0) {
				leftSide = true; // move left side up because it has slipped down
			}
			else {
				leftSide = false; // otherwise move right side up because it has slipped down
			}

			stepSlip++;
			break;
		}
		if (leftSide)
		{
			switch(stepSlip) {
			case 1:
				fix = std::min(abs(eGyroValue)/bufferZ, defP);
				lift_L->Set(1.0*(smoothStart + fix)*hardCorrection);

				if (((eGyroValue < gyroAlignThresh) || (abs(eGyroValue - 360.0) < gyroAlignThresh)) || toteTimer->Get() > 10.0) {
					stepSlip++;
					getToteLastTime += toteTimer->Get();
				}
				break;
			case 2:
				fakeZero_L = r_LiftEncoder + liftEncoder_L->Get();
				stepSlip++;
				break;
			}
		}
		else
		{
			switch(stepSlip) {
			case 1:
				fix = std::min(abs(eGyroValue)/bufferZ, defP);
				lift_R->Set(-1.0*(smoothStart + fix));

				if (((eGyroValue < gyroAlignThresh) || (abs(eGyroValue - 360.0) > gyroAlignThresh)) || toteTimer->Get() > 10.0) {
					stepSlip++;
					getToteLastTime += toteTimer->Get();
				}
				break;
			case 2:
				fakeZero_R = l_LiftEncoder - liftEncoder_R->Get();
				stepSlip++;
				break;
			}
		}

		switch (stepSlip)
		{
		case 3:
			slipCorrectRunning = false;
			stepSlip = 0;
			getToteLastTime = toteTimer->Get();
		}
	}


	bool ReleaseTote() {
		switch(stepRelTote)
		{
		case 0:
			toteTimer->Reset();
			toteTimer->Start();
			piston1->Set(DoubleSolenoid::kForward);
			stepRelTote++;
			break;
		case 1:
			if (toteTimer->Get() > toteExtendTime) {
				stepRelTote++;
				getToteLastTime += toteTimer->Get();
			}
			break;

		case 2:
			suctionCupsOn = false; // let go of tote
			if (toteTimer->Get() > getToteLastTime + 0.2) { // wait a tiny amount of time for tote to unseal
				stepRelTote++;
				getToteLastTime += toteTimer->Get();
			}
			break;

		case 3: // retract with tote
			piston1->Set(DoubleSolenoid::kReverse);
			if (toteTimer->Get() > getToteLastTime + toteExtendTime) { // takes longer to retract
				relToteRunning = false;
				stepRelTote = 0;
				getToteLastTime = 0.0; // reset to 0 for next run
				return true;
			}
			break;
		}
		return false;
	}

	bool AcqGetTote() {
		switch(stepGetTote)
		{
		case 0:
			toteTimer->Reset();
			toteTimer->Start();
			suctionCupsOn = true;
			stepGetTote++;
			break;
		case 1:
			if (toteTimer->Get() > getToteSuckTime) {
				stepGetTote++;
				getToteLastTime += toteTimer->Get();
			}
			break;

		case 2:
			piston1->Set(DoubleSolenoid::kForward);
			if (toteTimer->Get() > getToteLastTime + toteExtendTime) {
				stepGetTote++;
				getToteLastTime += toteTimer->Get();
			}
			break;

		case 3: // retract with tote
			piston1->Set(DoubleSolenoid::kReverse);
			if (toteTimer->Get() > getToteLastTime + toteExtendTime) { // takes longer to retract
				stepGetTote = 0;
				getToteLastTime = 0.0; // reset to 0 for next run
				return true;
			}
			break;
		}
		return false;
	}

	void AcqInitialize() {
		ultrasonic_L_dist = ultrasonic_L->GetRangeInches();
		ultrasonic_R_dist = ultrasonic_R->GetRangeInches();
		stepAcq = 0; // reset to first step
	}

	void AcqRoutine() {

		switch (stepAcq)
		{
		case 0: // strafe right until ultrasonic sees tote
			OutputStrafe((float)acqRunning, acqStrafeSpeed); // strafe right

			if (acqRunning == 1) // from left side
			{
				if (ultrasonic_L->GetRangeInches() < ultrasonic_L_dist - closerTote_dist) {
					r_FrontEncoder_1 = rightFrontEncoder->Get(); // store each encoder for reference for next step
					l_FrontEncoder_1 = leftFrontEncoder->Get();
					r_BackEncoder_1 = rightBackEncoder->Get();
					l_BackEncoder_1 = leftBackEncoder->Get();
					stepAcq++;
				}
			}
			else // from right side
			{
				if (ultrasonic_R->GetRangeInches() < ultrasonic_R_dist - closerTote_dist) {
					r_FrontEncoder_1 = rightFrontEncoder->Get(); // store each encoder for reference for next step
					l_FrontEncoder_1 = leftFrontEncoder->Get();
					r_BackEncoder_1 = rightBackEncoder->Get();
					l_BackEncoder_1 = leftBackEncoder->Get();
					stepAcq++;
				}
			}
			break;
		case 1: // strafe right or left to align with levels
			OutputStrafe(float(acqRunning), acqStrafeSpeed);

			r_FrontDelta = abs(rightFrontEncoder->Get() - r_FrontEncoder_1);
			l_FrontDelta = abs(leftFrontEncoder->Get() - l_FrontEncoder_1);
			r_BackDelta = abs(rightBackEncoder->Get() - r_BackEncoder_1);
			l_BackDelta = abs(leftBackEncoder->Get() - l_BackEncoder_1);

			avgStrafeTicks = 0.25*(r_FrontDelta + l_FrontDelta + r_BackDelta + l_BackDelta);

			if (avgStrafeTicks*driveInchesPerTick > levelAlign_dist[AcqIndex()]) {
				stepAcq++;
			}
			break;
		case 2: // move backward/forward until right distance

			if (ultrasonic_L->GetRangeInches() - toteDepth_dist > 0) {
				dirDepth = 1.0;
			}
			else {
				dirDepth = -1.0;
			}

			OutputStraightDrive(dirDepth, 0.5);
			if (abs(ultrasonic_L->GetRangeInches() - toteDepth_dist) < toteDepth_range) {
				firstCall = true;
				stepAcq++;
			}
			break;
		case 3:
			OutputAllDrive(0.0); // stop drive train
			Wait(.4);
			stepAcq++;
			break;
		case 4:
			if (AcqGetTote()) { // run AcqGetTote, if done, end everything and reset
				acqRunning = 0;
				stepAcq = 0;
			}
			break;

		default:
			while (autonTimer->Get() < 15.0){

			}
		}
	}

	float AlignComparison(float angleIs, float angleTo) { // if first is less than second, return 1.0
		if (angleIs < angleTo) {
			return 1.0;
		}
		return 0.0;
	}

	void AutoBasic() {
		switch (stepDrive) // for drive train
		{
		case 0:
			testAuto->Set(.7);
			if (autonTimer->Get() > 2.0){
				stepDrive++;
			}
			break;
		case 1:
			testAuto->Set(0.0);
			if (autonTimer->Get() > 4.0) {
				stepDrive++;
			}
			break;
		case 2:
			testAuto->Set(.5);
			if (autonTimer->Get() > 7.0) {
				stepDrive++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
			break;
		}
	}

	void AutoForward() { // the most basic default AUTO routine
		switch (stepDrive) // for drive train
		{
		case 0:
			OutputDistanceDrive(1.0, 40.0);
			if (autonTimer->Get() > 15.0){
				stepDrive++;
			}
			break;
		case 1:
			OutputAllDrive(0.0);
			if (autonTimer->Get() > 15.0) {
				stepDrive++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
			break;
		}

		switch (stepLift) // for elevator
		{
		case 0:
			if (autonTimer->Get() > 15.0){
				stepLift++;
			}
			break;

		default:
			while (autonTimer->Get() < 15.0) {

			}
			break;
		}
	}

	void AutoOneTote() {
		switch (stepDrive) // for drive train
		{
		case 0:
			if (AcqGetTote()) {
				stepDrive++;
			}
			break;
		case 1:
			OutputDistanceDrive(-1.0, 40.0);
			if (autonTimer->Get() > 15.0){
				stepDrive++;
			}
			break;
		case 2:
			OutputAllDrive(0.0);
			Wait(.4);
			stepDrive++;
			break;
		case 3:
			if (ReleaseTote()) {
				stepDrive++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
			break;
		}

		/*
		switch (stepPneum) // for drive train
		{
		case 0:

			if (autonTimer->Get() > 15.0){
				stepPneum++;
			}
			break;
		case 1:
			OutputAllDrive(0.0);
			if (autonTimer->Get() > 15.0) {
				stepPneum++;
			}
			break;
		default:
			while (autonTimer->Get() < 15.0){

			}
			break;
		} */

	}

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		gyro->InitGyro();
		eGyro->InitGyro();
	}

	void AutonomousInit()
	{
		//testEncoder->Reset();
		//int step = 0;
		//ResetDriveEncoders();
/*
		if (SmartDashboard::GetBoolean("DB/Button 0", false)) {
			autoNumber = 0;
		}
		else if (SmartDashboard::GetBoolean("DB/Button 1", false)) {
			autoNumber = 1;
		}
		else if (SmartDashboard::GetBoolean("DB/Button 2", false)) {
			autoNumber = 2;
		}
		else if (SmartDashboard::GetBoolean("DB/Button 3", false)) {
			autoNumber = 3;
		}

		stepDrive = 0;*/
		liftEncoder_L->Reset();
		liftEncoder_R->Reset();

		autonTimer = new Timer();
		autonTimer->Start();

		/*
		gyro->InitGyro();
		eGyro->InitGyro();
		gyroValue = 0;
		eGyroValue = 0; */
	}

	void AutonomousPeriodic()
	{
		//intermediateGyro = ((int)gyro->GetAngle() + 3600000) % 360;
		//gyroValue = (float)intermediateGyro; // it's a FLOAT

		/*
		l_LiftEncoder = -1*(liftEncoder_L->Get()) + fakeZero_L;
		r_LiftEncoder = (liftEncoder_R->Get()) + fakeZero_R;

		r_frontEncoder = rightFrontEncoder->Get();
		l_frontEncoder = leftFrontEncoder->Get();
		r_backEncoder = rightBackEncoder->Get();
		l_backEncoder = leftBackEncoder->Get();
*/
		//suctionCups->Set(suctionCupsOn);

		//AutoForward();

		drive->MecanumDrive_Cartesian(0.0,0.0,0.0);

		if (autonTimer->Get() < 3.5) {
			float s = 0.4;
			leftFront->Set(s);
			leftBack->Set(s);
			rightFront->Set(-s);
			rightBack->Set(-s);
		}
/*
		else if (autonTimer->Get() < 4.1)
		{
			float s = 0.4;
			leftFront->Set(s);
			leftBack->Set(s);
			rightFront->Set(s);
			rightBack->Set(s);
		}

		else if (autonTimer->Get() < 15.0){
			ResetElevator();
			float s = 0.0;
			leftFront->Set(s);
			leftBack->Set(s);
			rightFront->Set(s);
			rightBack->Set(s);
		} */
		else {
			float s = 0.0;
			leftFront->Set(s);
			leftBack->Set(s);
			rightFront->Set(s);
			rightBack->Set(s);
		}

//		leftFront->Set(PWMlf->Get());
//		leftBack->Set(PWMlb->Get());
//		rightFront->Set(PWMrf->Get());
//		rightBack->Set(PWMrb->Get());

		// for drivetrain
/*
		if (autonTimer->Get() < 15.0)
		{
			switch(autoNumber)
			{
			case 0:
				AutoForward();
				break;

			case 1:
				AutoOneTote();
				break;
			case 2:
				AutoBasic();

			}
		}
		*/

//		SmartDashboard::PutNumber("Left lift Encoder", liftEncoder_L->Get());
//		SmartDashboard::PutNumber("Right lift Encoder", liftEncoder_R->Get());
//		SmartDashboard::PutNumber("ultrasonic", ultrasonic_L->GetRangeInches());
		SmartDashboard::PutBoolean("Bottom Limit", bottomLimit_L->Get());
	}

	void TeleopInit()
	{
		//SmartDashboard::init();
		liftEncoder_L->Reset();
		liftEncoder_R->Reset();
		gyro->Reset();
		eGyro->Reset();
		gyroValue = 0;
		eGyroValue = 0;

		ResetDriveEncoders();

		ultrasonic_L->SetAutomaticMode(true);
		ultrasonic_R->SetAutomaticMode(true);

		//drive->SetSafetyEnabled(false);
		drive->SetInvertedMotor(RobotDrive::MotorType::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::MotorType::kRearRightMotor, true);
		drive->SetExpiration(1.0);

		leftFront->SetExpiration(1.0);
		leftBack->SetExpiration(1.0);
		rightFront->SetExpiration(1.0);
		rightBack->SetExpiration(1.0);
	}

	void TeleopPeriodic()
	{

		// Mecanum drive, no gyro
		/*
		rightFront->Set(speedM*(driveStick->GetY() - driveStick->GetZ() - driveStick->GetX()));
		rightBack->Set(speedM*(driveStick->GetY() - driveStick->GetZ() + driveStick->GetX()));
		leftFront->Set(speedM*(driveStick->GetY() + driveStick->GetZ() + driveStick->GetX()));
		leftBack->Set(speedM*(driveStick->GetY() + driveStick->GetZ() - driveStick->GetX()));
*/
/*
		leftFront->SetControlMode(CANSpeedController::kPercentVbus);
		leftBack->SetControlMode(CANSpeedController::kPercentVbus);
		rightFront->SetControlMode(CANSpeedController::kPercentVbus);
		rightBack->SetControlMode(CANSpeedController::kPercentVbus);
		drive->MecanumDrive_Cartesian(speedM*(driveStick->GetX()), speedM*(driveStick->GetY()), speedM*(driveStick->GetZ()));
*/
//		drive->MecanumDrive_Cartesian(driveStick->GetX(),driveStick->GetY(),driveStick->GetZ(), gyro->GetAngle());
//		leftFront->Set(PWMlf->Get());
//		leftBack->Set(PWMlb->Get());
//		rightFront->Set(PWMrf->Get());
//		rightBack->Set(PWMrb->Get());

		//Wait(cycleWaitTime);

		intermediateGyro = ((int)gyro->GetAngle() + 3600000) % 360;
		gyroValue = (float)intermediateGyro; // it's a FLOAT

		intermediateGyro = ((int)eGyro->GetAngle() + 3600000) % 360;
		eGyroValue = (float)intermediateGyro; // it's a FLOAT

		l_LiftEncoder = -1*(liftEncoder_L->Get()) + fakeZero_L;
		r_LiftEncoder = (liftEncoder_R->Get()) + fakeZero_R;

		r_frontEncoder = rightFrontEncoder->Get();
		l_frontEncoder = leftFrontEncoder->Get();
		r_backEncoder = rightBackEncoder->Get();
		l_backEncoder = leftBackEncoder->Get();

		suctionCups->Set(suctionCupsOn);

		if (pistonsOn){
			piston1->Set(DoubleSolenoid::kForward);
			//piston2->Set(DoubleSolenoid::kForward);
		} else {
			piston1->Set(DoubleSolenoid::kReverse);
			//piston2->Set(DoubleSolenoid::kReverse);
		}

		leftFront->Set(PWMlf->Get());
		leftBack->Set(PWMlb->Get());
		rightFront->Set(PWMrf->Get());
		rightBack->Set(PWMrb->Get());

		// user control select what autonomous programs to run

		if (driveStick->GetRawButton(getToteButton) && !slipCorrectRunning) {
			getToteRunning = true;
		}
		else if (driveStick->GetRawButton(relToteButton) && !getToteRunning && !acqRunning && !slipCorrectRunning) {
			relToteRunning = true;
		}
		else if (driveStick->GetPOV() == acqStart_L && !relToteRunning && !getToteRunning && !slipCorrectRunning) {
			acqRunning = 1;
			stepAcq = 0; // reset to beginning of routine
			AcqInitialize();
		}
		else if (driveStick->GetPOV() == acqStart_R && !relToteRunning && !getToteRunning && !slipCorrectRunning) {
			acqRunning = -1;
			stepAcq = 0; // reset to beginning of routine
			AcqInitialize();
		}

		if (driveStick->GetRawButton(killSwitch)) { // override and stop all autonomous routines
			acqRunning = 0; // stop running
			stepAcq = 0;    // reset to first step

			getToteRunning = false;
			stepGetTote = 0;

			relToteRunning = false;
			stepRelTote = 0;

			slipCorrectRunning = false;
			stepSlip = 0;
		}

		// run what has been pressed to run

		if (acqRunning != 0 && !slipCorrectRunning) {
			AcqRoutine();
		}
		else if (getToteRunning && !slipCorrectRunning) {
			if (AcqGetTote()) {
				getToteRunning = false;
			}
		}
		else if (relToteRunning && !slipCorrectRunning) {
			if (ReleaseTote()) {
				relToteRunning = false;
			}
		}
		else // else default to regular user control
		{
			if (driveStick->GetRawButton(northButton)) // north = 0, dir = 1 means clockwise is positive
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue > southDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign * (float)std::min((float)(abs((float)northDegrees + (float)360.0 - (float)gyroValue)/((float)alignBufferZone) + (float)minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn( (float)(-dir) * smoothAlign *(float)std::min((float)abs((float)northDegrees - (float)gyroValue)/(float)alignBufferZone + (float)minAlignMultiplier, (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(eastButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue > westDegrees || gyroValue < eastDegrees) {
					//OutputPointTurn(  (float)dir * (float)std::min( AlignComparison(gyroValue, northDegrees) * (float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
					OutputPointTurn(  (float)dir * smoothAlign *(float)std::min((float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else { // if between 90 and 270
					OutputPointTurn(  (float)(-dir) * smoothAlign *(float)std::min( (float)(abs( (eastDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(southButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue < southDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign *(float)std::min( (float)(abs( (southDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn(  (float)(-dir) * smoothAlign *(float)std::min( (float)(abs( (southDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else if (driveStick->GetRawButton(westButton))
			{
				if (smoothAlign < maxAlignSpeed) {
					smoothAlign += increaseAlignSpeed;
				}
				else {
					smoothAlign = maxAlignSpeed;
				}

				if (gyroValue < westDegrees && gyroValue > eastDegrees) {
					OutputPointTurn(  (float)dir * smoothAlign * (float)std::min( (float)(abs( (westDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
				else {
					OutputPointTurn(  (float)(-dir) * smoothAlign * (float)std::min( (float)(abs( (westDegrees - gyroValue)/(alignBufferZone)) + minAlignMultiplier), (float)1.0), (float)maxAlignSpeed);
				}
			}
			else {
				if (orientEnabled){
					drive->MecanumDrive_Cartesian(speedM*driveStick->GetX(),speedM*driveStick->GetY(),speedM*driveStick->GetZ()*turnSpeed, 0.0);
				} else {
					drive->MecanumDrive_Cartesian(speedM*driveStick->GetX(),speedM*driveStick->GetY(),speedM*driveStick->GetZ()*turnSpeed, gyro->GetAngle());
				}
				smoothAlign = startAlignSpeed;
				//OutputAllDrive(0.0);
			}

			if (driveStick->GetRawButton(resetGyroButton)) {
				gyro->Reset();
			}

			// Reset Encoders in elevator
			if (auxStick->GetRawButton(resetEncodersButton))
			{
				liftEncoder_L->Reset();
				liftEncoder_R->Reset();
			}

			// pneumatics

			// suction cups
			if (!auxStick->GetRawButton(suctionCupsButton)) {
				lastSuctionButton = true;
			}
			if (auxStick->GetRawButton(suctionCupsButton) && lastSuctionButton) {
				lastSuctionButton = false;
				suctionCupsOn = !suctionCupsOn;
			}

			// pneumatics arm extender
			if (!auxStick->GetRawButton(pistonButton)) {
				lastPistonButton = true;
			}
			if (auxStick->GetRawButton(pistonButton) && lastPistonButton) {
				lastPistonButton = false;
				pistonsOn = !pistonsOn;
			}

			if (!driveStick->GetRawButton(orientToggleButton)) {
							lastOrientButton = true;
			}
			if (auxStick->GetRawButton(orientToggleButton) && lastOrientButton) {
				lastOrientButton = false;
				orientEnabled = !orientEnabled;
			}

			// elevator lift code with levels, smooth start and smooth stop

			if (auxStick->GetRawButton(resetElevatorButton)) {
				ResetElevator();
			}

			else if(auxStick->GetPOV() == slipCorrectButton) {
				if(!slipCorrectRunning && ((abs(eGyroValue - 0.0) > gyroSlipThresh && eGyroValue < 80.0) || (abs(eGyroValue - 360.0) > gyroSlipThresh && eGyroValue > 280.0))
								     && abs(l_frontEncoder - r_frontEncoder) < encoSlipThresh) {

					slipCorrectRunning = true;
				}

				if(slipCorrectRunning) {
					SlipCorrect();
				}
			}
			else
			{
				resetDifference = correction*smoothStart*std::min((float)abs(l_LiftEncoder-r_LiftEncoder)/50.0 + 0.5,1.0);
				correctionDifference = correction*smoothStart*std::min((float)abs(l_LiftEncoder-r_LiftEncoder)/50.0 + 0.5,1.0);

				if ( (topLimit_L->Get()) && (auxStick->GetRawButton(upButton)
						|| (auxStick->GetRawButton(pos1Button) && l_LiftEncoder < 0)
						|| (auxStick->GetRawButton(pos2Button) && l_LiftEncoder < toteHeight)
						|| (auxStick->GetRawButton(pos3Button) && l_LiftEncoder < toteHeight*2)
						|| (auxStick->GetRawButton(pos4Button) && l_LiftEncoder < toteHeight*3))) // move up
				{
					if (smoothStart < maxLiftSpeed) {
						smoothStart += increaseSpeed;
					}
					else {
						smoothStart = maxLiftSpeed;
					}

					if (l_LiftEncoder < r_LiftEncoder) {
						OutputLift(1.0
								* std::min(abs( (abs(l_LiftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
						, correctionDifference);
					}
					else if (l_LiftEncoder > r_LiftEncoder) {
						OutputLift(1.0
								* std::min(abs( (abs(l_LiftEncoder)-(maxHeight - (maxHeight - toteHeight)*(int)auxStick->GetRawButton(pos2Button)) )/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
						, -correctionDifference);
					}
					else {
						OutputLift(1.0);
					}
				}
				else if ( (bottomLimit_L->Get()) && (auxStick->GetRawButton(downButton)
						|| (auxStick->GetRawButton(pos1Button) && l_LiftEncoder > 0)
						|| (auxStick->GetRawButton(pos2Button) && l_LiftEncoder > toteHeight)
						|| (auxStick->GetRawButton(pos3Button) && l_LiftEncoder > toteHeight*2)
						|| (auxStick->GetRawButton(pos4Button) && l_LiftEncoder > toteHeight*3))) // move down
				{
					if (smoothStart < maxLiftSpeed) {
						smoothStart += increaseSpeed;
					}
					else {
						smoothStart = maxLiftSpeed;
					}

					if (l_LiftEncoder > r_LiftEncoder) {
						OutputLift(-1.0
								* std::min(abs((abs(l_LiftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
						, correctionDifference);
					}
					else if (l_LiftEncoder < r_LiftEncoder) {
						OutputLift(-1.0
								* std::min(abs((abs(l_LiftEncoder)-toteHeight*(int)auxStick->GetRawButton(pos2Button))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 2*toteHeight)*(int)auxStick->GetRawButton(pos3Button)))/stopBuffer),1)
						* std::min(abs((abs(l_LiftEncoder)-(2*maxHeight - (2*maxHeight - 3*toteHeight)*(int)auxStick->GetRawButton(pos4Button)))/stopBuffer),1)
						, -correctionDifference);
					}
					else {
						OutputLift(-1.0);
					}
				}
			/*
			else if ((topLimit_L->Get()) && driveStick->GetRawButton(5)) { // move up
				OutputLiftRegular(1.0, speedM);
			}
			else if ((bottomLimit_L->Get()) && driveStick->GetRawButton(7)) { // move down
				OutputLiftRegular(-1.0, speedM);
			}*/
				else {
					//				OutputLift(0.0);

					lift_L->Set(-speedM*auxStick->GetY()); // defaults to regular joyauxStick control of each lift side
					lift_R->Set(speedM*auxStick->GetThrottle());

					smoothStart = startSpeed;
				}
			}
		}

		SmartDashboard::PutNumber("Lift Left", l_LiftEncoder);
		SmartDashboard::PutNumber("Lift Right", r_LiftEncoder);
		SmartDashboard::PutNumber("Gyro:", gyroValue);
		SmartDashboard::PutNumber("Elevator Gyro", eGyroValue);
		SmartDashboard::PutNumber("Elevator Gyro Raw", eGyro->GetAngle());
		SmartDashboard::PutNumber("Left Ultrasonic", ultrasonic_L->GetRangeInches());
		SmartDashboard::PutNumber("Ultrasonic", ultrasonic_R->GetRangeInches());

		SmartDashboard::PutNumber("Front Right", r_frontEncoder);
		SmartDashboard::PutNumber("Front Left", l_frontEncoder);
		SmartDashboard::PutNumber("Back Right", r_backEncoder);
		SmartDashboard::PutNumber("Back Left", l_backEncoder);
		SmartDashboard::PutBoolean("Bottom Limit", bottomLimit_L->Get());
		SmartDashboard::PutBoolean("Suction Cups On:", suctionCupsOn);
		SmartDashboard::PutBoolean("Pistons Extended:", pistonsOn);
		SmartDashboard::PutBoolean("SlipCorrect Running:", slipCorrectRunning);
		SmartDashboard::PutBoolean("OrientDrive On:", orientEnabled);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
