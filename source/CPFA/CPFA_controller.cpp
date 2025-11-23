#include "CPFA_controller.h"
#include <unistd.h>

CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	ResourceDensity(0),
	MaxTrailSize(50),
	SearchTime(0),
	CPFA_state(DEPARTING),
	LoopFunctions(NULL),
	survey_count(0),
	isUsingPheromone(0),
    SiteFidelityPosition(1000, 1000), 
    searchingTime(0),
    travelingTime(0),
    startTime(0),
    m_pcLEDs(NULL),
    TrailColor(CColor::BLUE),
    updateFidelity(false),
    last_time_in_seconds(0),
	// ---- NEW defaults ----
    s_WindowSize(100),
    sw_sample_pos(4),   // real ~0.25 s/sample at 32 TPS
    sw_waitTicks(1),
    sw_CongRatioOn(1.7),
    sw_bad_samples(5), // ~1.25 s of sustained congestion
    sw_congEps(0.02),
    sum_window_segments(0.0),
    sw_LastCongSampleTick(0),
    sw_badSample_counter(0),
    InCongested(false)
{
	// initialize restricted-zone bookkeeping
    hasRestrictedZone = false;
    zoneXMin = zoneXMax = zoneYMin = zoneYMax = 0.0;
}

void CPFA_controller::Init(argos::TConfigurationNode &node) {
	compassSensor   = GetSensor<argos::CCI_PositioningSensor>("positioning");
	wheelActuator   = GetActuator<argos::CCI_DifferentialSteeringActuator>("differential_steering");
	proximitySensor = GetSensor<argos::CCI_FootBotProximitySensor>("footbot_proximity");
	argos::TConfigurationNode settings = argos::GetNode(node, "settings");

	argos::GetNodeAttribute(settings, "FoodDistanceTolerance",   FoodDistanceTolerance);
	argos::GetNodeAttribute(settings, "TargetDistanceTolerance", TargetDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestDistanceTolerance", NestDistanceTolerance);
	argos::GetNodeAttribute(settings, "NestAngleTolerance",    NestAngleTolerance);
	argos::GetNodeAttribute(settings, "TargetAngleTolerance",    TargetAngleTolerance);
	argos::GetNodeAttribute(settings, "SearchStepSize",          SearchStepSize);
	argos::GetNodeAttribute(settings, "RobotForwardSpeed",       RobotForwardSpeed);
	argos::GetNodeAttribute(settings, "RobotRotationSpeed",      RobotRotationSpeed);
	argos::GetNodeAttribute(settings, "ResultsDirectoryPath",      results_path);
	argos::GetNodeAttribute(settings, "DestinationNoiseStdev",      DestinationNoiseStdev);
	argos::GetNodeAttribute(settings, "PositionNoiseStdev",      PositionNoiseStdev);

	// --- NEW: congestion params (keeps defaults if absent)
	argos::GetNodeAttributeOrDefault(settings, "CongWindowSize",    s_WindowSize,    s_WindowSize);
	argos::GetNodeAttributeOrDefault(settings, "CongSampleDivisor", sw_sample_pos, sw_sample_pos);
	argos::GetNodeAttributeOrDefault(settings, "CongRatioOn",      sw_CongRatioOn,       sw_CongRatioOn);
	argos::GetNodeAttributeOrDefault(settings, "CongHOn",           sw_bad_samples,         sw_bad_samples);
	argos::GetNodeAttributeOrDefault(settings, "CongEps",           sw_congEps,           sw_congEps);

	// Derived cadence (ticks)
	unsigned tps = SimulationTicksPerSecond();
	sw_waitTicks = std::max<size_t>(1, tps / std::max<size_t>(1, sw_sample_pos));
	sw_LastCongSampleTick = SimulationTick(); // start gate from "now"


	argos::CVector2 p(GetPosition());
	SetStartPosition(argos::CVector3(p.GetX(), p.GetY(), 0.0));
	
	FoodDistanceTolerance *= FoodDistanceTolerance;
	SetIsHeadingToNest(true);
	//qilu 10/21/2016 Let robots start to search immediately
	SetTarget(p);
        controllerID= GetId();
    m_pcLEDs   = GetActuator<CCI_LEDsActuator>("leds");
    controllerID= GetId();//qilu 07/26/2016
		m_pcLEDs->SetAllColors(CColor::GREEN);
}

void CPFA_controller::ControlStep() {
	/*
	ofstream log_output_stream;
	log_output_stream.open("cpfa_log.txt", ios::app);

	// depart from nest after food drop off or simulation start
	if (isHoldingFood) log_output_stream << "(Carrying) ";
	
	switch(CPFA_state)  {
		case DEPARTING:
			if (isUsingSiteFidelity) {
				log_output_stream << "DEPARTING (Fidelity): "
					<< GetTarget().GetX() << ", " << GetTarget().GetY()
					<< endl;
			} else if (isInformed) {
				log_output_stream << "DEPARTING (Waypoint): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			} else {
				log_output_stream << "DEPARTING (Searching): "
				<< GetTarget().GetX() << ", " << GetTarget().GetY() << endl;
			}
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			if (isInformed) log_output_stream << "SEARCHING: Informed" << endl;     
			else log_output_stream << "SEARCHING: UnInformed" << endl;
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			log_output_stream << "RETURNING" << endl;
			break;
		case SURVEYING:
			log_output_stream << "SURVEYING" << endl;
			break;
		default:
			log_output_stream << "Unknown state" << endl;
	}
	*/

	// Add line so we can draw the trail
	curr_time_in_seconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond()); 
     
	// if(curr_time_in_seconds - last_time_in_seconds >= 0)
	// {
	// 	CVector2 position2d(GetPosition().GetX(), GetPosition().GetY());
		
	// 	CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
	// 	CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
	// 	CRay3 targetRay(target3d, position3d);
	// 	myTrail.push_back(targetRay);
	// 	LoopFunctions->Trajectory[controllerID].push_back(position2d);
	// 	//since it costs a lot of memeory, I commented it. qilu 06/2023. You can uncomment it if you want to show the trails.
	// 	LoopFunctions->TargetRayList.push_back(targetRay);
	// 	LoopFunctions->TargetRayColorList.push_back(TrailColor);
	// 	//argos::LOG<< "TargetRayList size =" << LoopFunctions->TargetRayList.size() <<endl;
	// 	previous_position = GetPosition();
	// 	last_time_in_seconds = curr_time_in_seconds;
    //  }
	//UpdateTargetRayList();
	CPFA();
	Move();
}

void CPFA_controller::Reset() {
    num_targets_collected =0;
    isHoldingFood   = false;
    isInformed      = false;
    SearchTime      = 0;
    ResourceDensity = 0;
    RobotDensity = 0;
    collisionDelay = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
	
	// clear restricted search zone on reset
    hasRestrictedZone = false;
    
    
    /* Set LED color */
    /* m_pcLEDs->SetAllColors(CColor::BLACK); //qilu 09/04 */
    SetTarget(LoopFunctions->NestPosition); //qilu 09/08
    updateFidelity = false;
    TrailToShare.clear();
    TrailToFollow.clear();
    MyTrail.clear();

	myTrail.clear();

	isInformed = false;
	isHoldingFood = false;
	isUsingSiteFidelity = false;
	isGivingUpSearch = false;

	Cong_ResetWindow();
	InCongested = false;
	cooldownUntilTick = 0;
}

bool CPFA_controller::IsHoldingFood() {
		return isHoldingFood;
}

bool CPFA_controller::IsUsingSiteFidelity() {
		return isUsingSiteFidelity;
}

void CPFA_controller::CPFA() {
	
	switch(CPFA_state) {
		// depart from nest after food drop off or simulation start
		case DEPARTING:
			//argos::LOG << "DEPARTING" << std::endl;
			//SetIsHeadingToNest(false);
			Departing();
			break;
		// after departing(), once conditions are met, begin searching()
		case SEARCHING:
			//argos::LOG << "SEARCHING" << std::endl;
			//SetIsHeadingToNest(false);
			if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
				Searching();
			}
			break;
		// return to nest after food pick up or giving up searching()
		case RETURNING:
			//argos::LOG << "RETURNING" << std::endl;
			//SetIsHeadingToNest(true);
			Returning();
			break;
		case SURVEYING:
			//argos::LOG << "SURVEYING" << std::endl;
			//SetIsHeadingToNest(false);
			Surveying();
			break;
		
		case CONGESTED:
    		Congested();
    		break;
	}
}

bool CPFA_controller::IsInTheNest() {
    
	return ((GetPosition() - LoopFunctions->NestPosition).SquareLength()
		< LoopFunctions->NestRadiusSquared);
	}

void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;

	// Initialize the SiteFidelityPosition

	// Create the output file here because it needs LoopFunctions
		
	// Name the results file with the current time and date
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	stringstream ss;

	char hostname[1024];                                                   
	hostname[1023] = '\0';                    
	gethostname(hostname, 1023);  

	/* ss << "CPFA-"<<GIT_BRANCH<<"-"<<GIT_COMMIT_HASH<<"-"
		<< hostname << '-'
		<< getpid() << '-'
		<< (now->tm_year) << '-'
		<< (now->tm_mon + 1) << '-'
		<<  now->tm_mday << '-'
		<<  now->tm_hour << '-'
		<<  now->tm_min << '-'
		<<  now->tm_sec << ".csv";

		string results_file_name = ss.str();
		results_full_path = results_path+"/"+results_file_name;
         */
         
	// Only the first robot should do this:	 
	if (GetId().compare("CPFA_0") == 0) {
		/*
		ofstream results_output_stream;
		results_output_stream.open(results_full_path, ios::app);
		results_output_stream << "NumberOfRobots, "
			<< "TargetDistanceTolerance, "
			<< "TargetAngleTolerance, "
			<< "FoodDistanceTolerance, "
			<< "RobotForwardSpeed, "
			<< "RobotRotationSpeed, "
			<< "RandomSeed, "
			<< "ProbabilityOfSwitchingToSearching, "
			<< "ProbabilityOfReturningToNest, "
			<< "UninformedSearchVariation, "   
			<< "RateOfInformedSearchDecay, "   
			<< "RateOfSiteFidelity, "          
			<< "RateOfLayingPheromone, "       
			<< "RateOfPheromoneDecay" << endl
			<< LoopFunctions->getNumberOfRobots() << ", "
			<< CSimulator::GetInstance().GetRandomSeed() << ", "  
			<< TargetDistanceTolerance << ", "
			<< TargetAngleTolerance << ", "
			<< FoodDistanceTolerance << ", "
			<< RobotForwardSpeed << ", "
			<< RobotRotationSpeed << ", "
			<< LoopFunctions->getProbabilityOfSwitchingToSearching() << ", "
			<< LoopFunctions->getProbabilityOfReturningToNest() << ", "
			<< LoopFunctions->getUninformedSearchVariation() << ", "
			<< LoopFunctions->getRateOfInformedSearchDecay() << ", "
			<< LoopFunctions->getRateOfSiteFidelity() << ", "
			<< LoopFunctions->getRateOfLayingPheromone() << ", "
			<< LoopFunctions->getRateOfPheromoneDecay()
			<< endl;
				
			results_output_stream.close();
		*/
	}

}

void CPFA_controller::Congested() {
     // One-tick action: drop carried resource due to congestion, then switch to SEARCHING

    // 1) Re-add item at current position (drop)
    LoopFunctions->FoodList.push_back(GetPosition());
    LoopFunctions->FoodColoringList.push_back(argos::CColor::RED);

    // 2) No longer carrying
    isHoldingFood = false;

	// 1b) define restricted search rectangle based on drop position and arena bounds
   {
       argos::Real px = GetPosition().GetX();
       argos::Real py = GetPosition().GetY();
       argos::Real Xmin = ForageRangeX.GetMin();
       argos::Real Xmax = ForageRangeX.GetMax();
       argos::Real Ymin = ForageRangeY.GetMin();
       argos::Real Ymax = ForageRangeY.GetMax();

       if(px >= 0.0) { zoneXMin = px; zoneXMax = Xmax; }
       else          { zoneXMin = Xmin; zoneXMax = px; }

       if(py >= 0.0) { zoneYMin = py; zoneYMax = Ymax; }
       else          { zoneYMin = Ymin; zoneYMax = py; }

       hasRestrictedZone = true;
   }

    // 4) Switch to DEPARTING 
	// *******
	// 	argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
	//     argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
	//     if(updateFidelity && poissonCDF_sFollowRate > r2) {
	// 	    //log_output_stream << "Using site fidelity" << endl;
	// 	        SetIsHeadingToNest(false);
	// 	        SetTarget(SiteFidelityPosition);
	// 	        isInformed = true;
	//     }
	// 	*****
    //   // use pheromone waypoints
    //   else if(SetTargetPheromone()) {
    //       //log_output_stream << "Using site pheremone" << endl;
    //       isInformed = true;
    //       isUsingSiteFidelity = false;
    //   }
       // use random search
    //   **else {
    //        //log_output_stream << "Using random search" << endl;
    //        ** SetRandomSearchLocation();
    //         **isInformed = false;
    //        ** isUsingSiteFidelity = false;
    //   }
       /* this trip must be uninformed */
   isInformed = false;
   isUsingSiteFidelity = false;
   isGivingUpSearch = false;
   // Pick a new random target (this will use the restricted zone)
	SetRandomSearchLocation();
    CPFA_state = DEPARTING;
	SetIsHeadingToNest(false);
   // 5) Reset congestion detector bookkeeping
    Cong_ResetWindow();
	InCongested = false;

 // LEDs + log: restricted departing
   if(m_pcLEDs)
       m_pcLEDs->SetAllColors(CColor::RED); // DEPARTING (restricted)

   LOG << "[" << GetId() << "] RESTRICTED TRIP: ENTER DEPARTING at t="
       << (argos::Real)SimulationTick() / SimulationTicksPerSecond()
       << " pos=" << GetPosition()
       << " zone=[" << zoneXMin << "," << zoneXMax
       << "]x[" << zoneYMin << "," << zoneYMax << "]\n";
}
//resets all congestion track history for the next cycle
void CPFA_controller::Cong_ResetWindow() {
    sw_positions.clear();
    sum_window_segments = 0.0;
    sw_LastCongSampleTick = SimulationTick();
    sw_badSample_counter = 0;
}
//starts when enough samples are collected
bool CPFA_controller::Cong_WindowFull() const {
    return sw_positions.size() >= s_WindowSize;
}
//computes tortuosity
argos::Real CPFA_controller::Cong_CurrentTortuosity() const {
    if (sw_positions.size() < 2) return 1.0;
    const argos::CVector2& a = sw_positions.front();
    const argos::CVector2& b = sw_positions.back();
    argos::Real euclid = (b - a).Length(); //euclidean distance
    if (euclid < sw_congEps) return 1.0;  // avoids errors when robots barely moved
    return sum_window_segments / euclid;
}
//mark that the robot is congested
void CPFA_controller::Cong_Enter() {
    InCongested = true;
    LOG << GetId() << " is CONGESTED (t= " 
        << (argos::Real)SimulationTick() / (argos::Real)SimulationTicksPerSecond() << "s)\n";
}
//samples position on a cadence, maintains window, compute tortuosity, updates hysteresis and trigger enter/exit
void CPFA_controller::Cong_TrySampleAndUpdate() {
    // Only track while returning (or already congested) AND carrying food
    if (CPFA_state != RETURNING && CPFA_state != CONGESTED) return;
    if (!isHoldingFood) return;
	//sample at a fixed rate
    const size_t now = SimulationTick();
    if (now - sw_LastCongSampleTick < sw_waitTicks) return; // cadence gate
    sw_LastCongSampleTick = now;

    // Sample current 2D position
    argos::CVector2 cur(GetPosition().GetX(), GetPosition().GetY());

    if (!sw_positions.empty()) {
        sum_window_segments += (cur - sw_positions.back()).Length();
    }
    sw_positions.push_back(cur);

    // Maintain sliding window
    if (sw_positions.size() > s_WindowSize) {
        const argos::CVector2 old0 = sw_positions.front();
        sw_positions.pop_front();
        const argos::CVector2 new0 = sw_positions.front();
        sum_window_segments -= (new0 - old0).Length();
        if (sum_window_segments < 0) sum_window_segments = 0; // numeric guard
    }
	//until window is full
    if (!Cong_WindowFull()) return;
	//until enough samples
    const argos::Real tau = Cong_CurrentTortuosity();

    // Hysteresis counter
    // Entry-only counters: count consecutive "bad" samples; reset otherwise
    if (tau >= sw_CongRatioOn) { // bad
        ++sw_badSample_counter;
    } else {
        sw_badSample_counter = 0;
    }

    // Transitions
    if (!InCongested && sw_badSample_counter >= sw_bad_samples) {
        CPFA_state = CONGESTED;
        Cong_Enter();
        return;
    }

}


void CPFA_controller::Departing()
{
    argos::Real distance = (GetPosition() - GetTarget()).Length();
    argos::Real randomNumber =
        RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

    argos::CVector2 target = GetTarget();
    argos::Real wallBuffer = 0.25;

    bool nearWall =
        (target.GetX() > ForageRangeX.GetMax() - wallBuffer ||
         target.GetX() < ForageRangeX.GetMin() + wallBuffer ||
         target.GetY() > ForageRangeY.GetMax() - wallBuffer ||
         target.GetY() < ForageRangeY.GetMin() + wallBuffer);

    argos::Real tolerance =
        nearWall ? TargetDistanceTolerance * 4.0 : TargetDistanceTolerance;

    /* ========================================================
       ==========  RESTRICTED (CONGESTION) OUTBOUND  ==========
       ======================================================== */
    if(hasRestrictedZone)
    {
        if(distance < tolerance) //robot reached restricted target
        {
            // Switch to SEARCHING
            Stop();
            CPFA_state = SEARCHING;
            SearchTime = 0;
            travelingTime += SimulationTick() - startTime;
            startTime = SimulationTick();
            SetIsHeadingToNest(false);

            /* ---- First random uninformed step inside zone ---- */
            argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
            argos::Real r = RNG->Gaussian(USV);

            argos::CRadians rotation(r);
            argos::CRadians heading = GetHeading();
			argos::CRadians turn_angle = heading + rotation;

            argos::CVector2 cand(SearchStepSize, turn_angle);
            cand += GetPosition();
			//normal cpfa behavior but applied inside restricted zone

            /* Clamp to restricted zone */
            if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
            else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);

            if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
            else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);

			
			if(hasRestrictedZone &&
			(cand.GetX() == zoneXMin || cand.GetX() == zoneXMax ||
				cand.GetY() == zoneYMin || cand.GetY() == zoneYMax))
			{
				SetRandomSearchLocation();
				return;
			}

            /* Anti-freeze: if too small step, nudge forward */
			//->>If the first step after departing hits the border of the rectangle: pick a new random point inside the quadrant
            argos::CVector2 cur = GetPosition();
            if((cand - cur).SquareLength() < 1e-4)
            {
                cand = cur + argos::CVector2(SearchStepSize, heading);

                // clamp again
				//->random step is effectively “0 movement": nudge it forward in its current heading
                if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
                else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);
                if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
                else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);
            }

            SetTarget(cand);

            if(m_pcLEDs)
                m_pcLEDs->SetAllColors(CColor::GREEN);

        }

        return; // ← critical
    }

    /* ========================================================
       ================  NORMAL DEPARTING  =====================
       ======================================================== */

    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0 &&
       !isInformed)
    {
        /* Random switch-to-searching */
        if(SimulationTick() % (5 * SimulationTicksPerSecond()) == 0 &&
           randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching)
        {
            Stop();
            CPFA_state = SEARCHING;
            SearchTime = 0;
            travelingTime += SimulationTick() - startTime;
            startTime = SimulationTick();

            if(m_pcLEDs)
                m_pcLEDs->SetAllColors(CColor::GREEN);

            argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
            argos::Real r = RNG->Gaussian(USV);

            argos::CRadians turn = GetHeading() + argos::CRadians(r);
            argos::CVector2 cand(SearchStepSize, turn);

            SetIsHeadingToNest(false);
            SetTarget(cand + GetPosition());
        }
        else if(distance < tolerance)
        {
            SetRandomSearchLocation();
        }
    }

    /* Informed search → start searching when we reach target */
    if(isInformed && distance < tolerance)
    {
        CPFA_state = SEARCHING;
        SearchTime = 0;
        travelingTime += SimulationTick() - startTime;
        startTime = SimulationTick();

        if(isUsingSiteFidelity)
        {
            isUsingSiteFidelity = false;
            SetFidelityList();
        }
    }
}


void CPFA_controller::Searching()
{
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0)
        SetHoldingFood();

    if(IsHoldingFood())
        return;

    argos::CVector2 cur = GetPosition();
    argos::CVector2 tgt = GetTarget();
    argos::Real dist = (cur - tgt).Length();

    /* ================================================================
       ==================   RESTRICTED ZONE SEARCHING   ===============
       ================================================================ */
    if(hasRestrictedZone)
    {
        if(m_pcLEDs)
            m_pcLEDs->SetAllColors(CColor::GREEN); // SEARCHING (restricted)

        LOG << "[" << GetId() << "] RESTRICTED TRIP: SEARCHING at t="
            << (argos::Real)SimulationTick() / SimulationTicksPerSecond()
            << " pos=" << cur
            << " target=" << GetTarget() << "\n";

        argos::CVector2 distance = cur - GetTarget();
        argos::Real dist2 = distance.Length();

        /* Anti-freeze: if target == position */
        if(dist2 < 1e-4)
        {
            argos::CVector2 cand =
                cur + argos::CVector2(SearchStepSize, GetHeading());

            if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
            else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);
            if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
            else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);

            SetTarget(cand);
            return;
        }

        argos::Real tolerance = TargetDistanceTolerance;

        if(dist2 < tolerance)
        {
            argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
            argos::Real rr = RNG->Gaussian(USCV);

            argos::CRadians turn = GetHeading() + argos::CRadians(rr);
            argos::CVector2 cand(SearchStepSize, turn);
            cand += cur;

            if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
            else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);
            if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
            else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);

            if((cand - cur).SquareLength() < 1e-4)
            {
                cand = cur + argos::CVector2(SearchStepSize, GetHeading());

                if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
                else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);
                if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
                else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);
            }

            /* leave restricted zone after first search step */
            hasRestrictedZone = false;

            SetIsHeadingToNest(false);
            SetTarget(cand);
        }

        return;  // <<< DO NOT TOUCH THE NORMAL CPFA CODE BELOW
    }


/* ==================================================================
   ===================  ORIGINAL CPFA SEARCHING  ====================
   ================================================================== */

 //LOG<<"Searching..."<<endl;
	// "scan" for food only every half of a second
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		SetHoldingFood();
	}
	// When not carrying food, calculate movement.
	if(IsHoldingFood() == false) {
		   argos::CVector2 distance = GetPosition() - GetTarget();
		   argos::Real     random   = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
     
       // If we reached our target search location, set a new one. The 
       // new search location calculation is different based on whether
       // we are currently using informed or uninformed search.
	   argos::CVector2 target = GetTarget();
       argos::Real wallBuffer = 0.25; // Distance to consider "near wall"
       bool nearWall = (target.GetX() > ForageRangeX.GetMax() - wallBuffer || 
                       target.GetX() < ForageRangeX.GetMin() + wallBuffer || 
                       target.GetY() > ForageRangeY.GetMax() - wallBuffer || 
                       target.GetY() < ForageRangeY.GetMin() + wallBuffer);

       argos::Real tolerance = nearWall ? TargetDistanceTolerance * 4.0 : TargetDistanceTolerance;
       if(distance.SquareLength() < tolerance) {
         // randomly give up searching
         if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest) {
             
             SetFidelityList();
	         TrailToShare.clear();
             SetIsHeadingToNest(true);
             SetTarget(LoopFunctions->NestPosition);
             isGivingUpSearch = true;
	         LoopFunctions->FidelityList.erase(controllerID);
             isUsingSiteFidelity = false; 
             updateFidelity = false; 
             CPFA_state = RETURNING;
             searchingTime+=SimulationTick()-startTime;
             startTime = SimulationTick();
             return; 
         }

         argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
         argos::Real rand = RNG->Gaussian(USCV);

         // uninformed search
         if(isInformed == false) {
          argos::CRadians rotation(rand);
          argos::CRadians angle1(rotation);
          argos::CRadians angle2(GetHeading());
          argos::CRadians turn_angle(angle1 + angle2);
          argos::CVector2 turn_vector(SearchStepSize, turn_angle);
          SetIsHeadingToNest(false);
          SetTarget(turn_vector + GetPosition());
         }
         // informed search
         else{
              SetIsHeadingToNest(false);
                  size_t          t           = SearchTime++;
                  argos::Real     twoPi       = (argos::CRadians::TWO_PI).GetValue();
                  argos::Real     pi          = (argos::CRadians::PI).GetValue();
                  argos::Real     isd         = LoopFunctions->RateOfInformedSearchDecay;
	              Real correlation = GetExponentialDecay(rand, t, isd);
	              argos::CRadians rotation(GetBound(correlation, -pi, pi));
                  argos::CRadians angle1(rotation);
                  argos::CRadians angle2(GetHeading());
                  argos::CRadians turn_angle(angle2 + angle1);
                  argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                  SetTarget(turn_vector + GetPosition());
         }
	  } 
    }
}


// Cause the robot to rotate in place as if surveying the surrounding targets
// Turns 36 times by 10 degrees
void CPFA_controller::Surveying() {
 //LOG<<"Surveying..."<<endl;
	if (survey_count <= 4) { 
		CRadians rotation(survey_count*3.14/2); // divide by 10 so the vector is small and the linear motion is minimized
		argos::CVector2 turn_vector(SearchStepSize, rotation.SignedNormalize());
		
		SetIsHeadingToNest(true); // Turn off error for this
		SetTarget(turn_vector + GetPosition());
		
		if(fabs((GetHeading() - rotation).SignedNormalize().GetValue()) < TargetAngleTolerance.GetValue()) survey_count++;
			//else Keep trying to reach the turning angle
	}
	// Set the survey countdown
	else {
		hasRestrictedZone = false;
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->NestPosition); 
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
        searchingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22
            
	}
}


/*****
 * RETURNING: Stay in this state until the robot has returned to the nest.
 * This state is triggered when a robot has found food or when it has given
 * up on searching and is returning to the nest.
 *****/
void CPFA_controller::Returning() {
 //LOG<<"Returning..."<<endl;
	//SetHoldingFood();
	Cong_TrySampleAndUpdate();
	// Are we there yet? (To the nest, that is.)
	if(IsInTheNest()) {
		// Based on a Poisson CDF, the robot may or may not create a pheromone
	    // located at the last place it picked up food.
	    argos::Real poissonCDF_pLayRate    = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfLayingPheromone);
	    argos::Real poissonCDF_sFollowRate = GetPoissonCDF(ResourceDensity, LoopFunctions->RateOfSiteFidelity);
	    argos::Real r1 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
	    argos::Real r2 = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
	    if (isHoldingFood) { 
          //drop off the food and display in the nest 
          //argos::CVector2 placementPosition;
          //placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
          
          //while((placementPosition-LoopFunctions->NestPosition).SquareLength()>pow(LoopFunctions->NestRadius/2.0-LoopFunctions->FoodRadius, 2))
            //  placementPosition.Set(LoopFunctions->NestPosition.GetX()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5), LoopFunctions->NestPosition.GetY()+RNG->Gaussian(LoopFunctions->NestRadius/1.2, 0.5));
     
          //LoopFunctions->CollectedFoodList.push_back(placementPosition);
          //Update the location of the nest qilu 09/10
          num_targets_collected++;
          //argos::LOG <<"num_targets_collected = "<<num_targets_collected<< endl;
		  LoopFunctions->currNumCollectedFood++;
          LoopFunctions->setScore(num_targets_collected);
          if(poissonCDF_pLayRate > r1 && updateFidelity) {
	            TrailToShare.push_back(LoopFunctions->NestPosition); //qilu 07/26/2016
                argos::Real timeInSeconds = (argos::Real)(SimulationTick() / SimulationTicksPerSecond());
		        Pheromone sharedPheromone(SiteFidelityPosition, TrailToShare, timeInSeconds, LoopFunctions->RateOfPheromoneDecay, ResourceDensity);
                LoopFunctions->PheromoneList.push_back(sharedPheromone);
                sharedPheromone.Deactivate(); // make sure this won't get re-added later...
                //argos::LOG <<"TrailToShare size =" << TrailToShare.size() << endl;
                //argos::LOG <<"LoopFunctions->PheromoneList size =" << LoopFunctions->PheromoneList.size() << endl;
          }
          TrailToShare.clear();  
	    }

	    // Determine probabilistically whether to use site fidelity, pheromone
	    // trails, or random search.
	    //ofstream log_output_stream;
	    //log_output_stream.open("cpfa_log.txt", ios::app);
	    //log_output_stream << "At the nest." << endl;	    
	 
	    // use site fidelity
	    if(updateFidelity && poissonCDF_sFollowRate > r2) {
		    //log_output_stream << "Using site fidelity" << endl;
		        SetIsHeadingToNest(false);
		        SetTarget(SiteFidelityPosition);
		        isInformed = true;
	    }
      // use pheromone waypoints
      else if(SetTargetPheromone()) {
          //log_output_stream << "Using site pheremone" << endl;
          isInformed = true;
          isUsingSiteFidelity = false;
      }
       // use random search
      else {
           //log_output_stream << "Using random search" << endl;
            SetRandomSearchLocation();
            isInformed = false;
            isUsingSiteFidelity = false;
      }

		isGivingUpSearch = false;
		CPFA_state = DEPARTING;   
        isHoldingFood = false; 
        travelingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22

		 // Clear restricted zone when robot returns to nest
        hasRestrictedZone = false;

        // --- CONGESTION: end-of-return cleanup ---
        Cong_ResetWindow();
        InCongested = false;
        sw_badSample_counter = 0;
        // ----------------------------------------

                
    } // end of In the nest
	// Take a small step towards the nest so we don't overshoot by too much if we miss it
    else 
    {
        if(IsAtTarget())
        {
	        //argos::LOG<<"heading to true in returning"<<endl;
	        //SetIsHeadingToNest(false); // Turn off error for this
	        //SetTarget(LoopFunctions->NestPosition);
	        //randomly search for the nest
	        argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
	        argos::Real rand = RNG->Gaussian(USCV);
	
	        argos::CRadians rotation(rand);
	        argos::CRadians angle1(rotation);
	        argos::CRadians angle2(GetHeading());
	        argos::CRadians turn_angle(angle1 + angle2);
	        argos::CVector2 turn_vector(SearchStepSize, turn_angle);
	        SetIsHeadingToNest(false);
	        SetTarget(turn_vector + GetPosition());
        }
        //detect other robots in its camera view
		if(SimulationTick()% SimulationTicksPerSecond() ==0 ){
				
			
	    }
	    
    }		
}


void CPFA_controller::SetRandomSearchLocation() {

	
    if(hasRestrictedZone) {
        if(zoneXMin >= zoneXMax - 0.05 || zoneYMin >= zoneYMax - 0.05) {
            hasRestrictedZone = false;
        }
    }

    /* ----------------------------------------------------------
       CASE 1: RESTRICTED QUADRANT (created by congestion)
       ---------------------------------------------------------- */
    if(hasRestrictedZone) {

        if(zoneXMin < zoneXMax && zoneYMin < zoneYMax) {

            /* Pick a uniform random location INSIDE the rectangle */
            argos::Real x = RNG->Uniform(argos::CRange<argos::Real>(zoneXMin, zoneXMax));
            argos::Real y = RNG->Uniform(argos::CRange<argos::Real>(zoneYMin, zoneYMax));

            argos::CVector2 cand(x, y);
            argos::CVector2 cur = GetPosition();

            /* --- 🔧 Anti-freeze safeguard ---
               Avoid target being too close to current position */
            argos::Real dx = cand.GetX() - cur.GetX();
            argos::Real dy = cand.GetY() - cur.GetY();
            if(dx*dx + dy*dy < 1e-4) {   // too close → nudge
                cand = cur + argos::CVector2(SearchStepSize, GetHeading());

                // clamp again (always required)
                if(cand.GetX() < zoneXMin) cand.SetX(zoneXMin);
                else if(cand.GetX() > zoneXMax) cand.SetX(zoneXMax);
                if(cand.GetY() < zoneYMin) cand.SetY(zoneYMin);
                else if(cand.GetY() > zoneYMax) cand.SetY(zoneYMax);
            }

            SetIsHeadingToNest(false);
            SetTarget(cand);

            if(m_pcLEDs)
                m_pcLEDs->SetAllColors(CColor::ORANGE);  // restricted outbound debug color

            return;
        }

        // If rectangle invalid → fall through to normal wall search
    }

    /* ----------------------------------------------------------
       CASE 2: ORIGINAL WALL-BASED RANDOM TARGET
       ---------------------------------------------------------- */
    argos::Real random_wall = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));
    argos::Real x = 0.0, y = 0.0;

    if(random_wall < 0.25) {
        x = RNG->Uniform(ForageRangeX);
        y = ForageRangeY.GetMax();      // north
    }
    else if(random_wall < 0.5) {
        x = RNG->Uniform(ForageRangeX);
        y = ForageRangeY.GetMin();      // south
    }
    else if(random_wall < 0.75) {
        x = ForageRangeX.GetMax();      
        y = RNG->Uniform(ForageRangeY); // east
    }
    else {
        x = ForageRangeX.GetMin();
        y = RNG->Uniform(ForageRangeY); // west
    }

    argos::CVector2 cand(x, y);
    argos::CVector2 cur = GetPosition();

    // /* --- 🔧 Anti-freeze for wall search ---
    //    Rare but safe to include */
    // argos::Real dx = cand.GetX() - cur.GetX();
    // argos::Real dy = cand.GetY() - cur.GetY();
    // if(dx*dx + dy*dy < 1e-4) {
    //     cand = cur + argos::CVector2(SearchStepSize, GetHeading());
    // }

    SetIsHeadingToNest(false);
    SetTarget(cand);

    if(m_pcLEDs)
        m_pcLEDs->SetAllColors(CColor::BLUE);  // normal outbound debug color
}



 
void CPFA_controller::SetHoldingFood() {
	// Is the iAnt already holding food?
	if(IsHoldingFood() == false) {
		// No, the iAnt isn't holding food. Check if we have found food at our
		// current position and update the food list if we have.

		    std::vector<argos::CVector2> newFoodList;
		    std::vector<argos::CColor> newFoodColoringList;
		    size_t i = 0, j = 0;
		    //argos::LOG<<"LoopFunctions->FoodList size =" <<LoopFunctions->FoodList.size() << endl;
		    //argos::LOG<<"LoopFunctions->FoodColoringList size =" <<LoopFunctions->FoodColoringList.size() << endl;
 
	         for(i = 0; i < LoopFunctions->FoodList.size(); i++) {
		            
	            if((GetPosition() - LoopFunctions->FoodList[i]).SquareLength() < FoodDistanceTolerance ) {
					// We found food! Calculate the nearby food density.
					 isHoldingFood = true;
                     CPFA_state = SURVEYING;
					 if(m_pcLEDs)
						m_pcLEDs->SetAllColors(CColor::CYAN); // RETURNING (will head nest)

					// LOG << "[" << GetId() << "] RESTRICTED TRIP: FOUND FOOD → exiting quadrant, heading nest (SURVEYING/RETURNING) t="
					// 	<< (argos::Real)SimulationTick() / SimulationTicksPerSecond()
					// 	<< " pos=" << GetPosition() << "\n";
					 j = i + 1;
					 searchingTime+=SimulationTick()-startTime;
					 startTime = SimulationTick();
					 
				   //distribute a new food 
			       /*  argos::CVector2 placementPosition;
			         placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
			          
			         while(LoopFunctions->IsOutOfBounds(placementPosition, 1, 1)){
			             placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
			         }
			         newFoodList.push_back(placementPosition);
					 newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
                    LoopFunctions->increaseNumDistributedFoodByOne(); //the total number of cubes in the arena should be updated. qilu 11/15/2018
					 //end
					 */
                     break; 
		         } else {
				   //Return this unfound-food position to the list
				   newFoodList.push_back(LoopFunctions->FoodList[i]);
				   newFoodColoringList.push_back(LoopFunctions->FoodColoringList[i]);
				 }
			 }
    
      if(j>0){
          for(; j < LoopFunctions->FoodList.size(); j++) {
              newFoodList.push_back(LoopFunctions->FoodList[j]);
              newFoodColoringList.push_back(LoopFunctions->FoodColoringList[j]);
          }
      }
		//argos::LOG<<"newFoodList size =" << newFoodList.size() << endl;
      // We picked up food. Update the food list minus what we picked up.
      if(IsHoldingFood()) {
         //SetIsHeadingToNest(true);
         //SetTarget(LoopFunctions->NestPosition);
         LoopFunctions->FoodList = newFoodList;
         LoopFunctions->FoodColoringList = newFoodColoringList; //qilu 09/12/2016
         SetLocalResourceDensity();
        
      }
      newFoodList.clear();
     newFoodColoringList.clear();
	}
	 
		
	// This shouldn't be checked here ---
	// Drop off food: We are holding food and have reached the nest.
	//else if((GetPosition() - LoopFunctions->NestPosition).SquareLength() < LoopFunctions->NestRadiusSquared) {
	//    isHoldingFood = false;
	// }

	// We are carrying food and haven't reached the nest, keep building up the
	// pheromone trail attached to this found food item.
  /*if(IsHoldingFood() && SimulationTick() % LoopFunctions->DrawDensityRate == 0) {
        TrailToShare.push_back(GetPosition());
  }*/
}

void CPFA_controller::SetRobotDensity() {
	argos::CVector2 vect1, vect2;
	RobotDensity = 0;
    argos::Real  lowerPt, upperPt;
    bool split; // the angle is in [-pi, pi]. If the camera range on both sides of the angle pi, we need to split the range into two ranges.
    argos::Real neighborAngle, targetAngle; 
    /* Calculate resource density based on the global food list positions. */
	
	for(map<string, CVector2>::iterator it= LoopFunctions->robotPosList.begin(); it!=LoopFunctions->robotPosList.end(); ++it){
		if(controllerID.compare(it->first) != 0){
		  vect1 = it->second - GetPosition();
		  //argos::LOG<<controllerID<<" current position= "<< GetPosition()<<endl;
		  //argos::LOG<<"Heading = "<< GetHeading().GetValue()<< endl;
		  
		  //argos::CRadians angle(GetHeading());
		  //lowerRange = (GetHeading().UnsignedNormalize() - (argos::CRadians::PI)/4.0).GetValue();  
		  split=false;    
	      lowerPt = (GetHeading() - (argos::CRadians::PI)/4.0).GetValue();      
	      upperPt = (GetHeading() + (argos::CRadians::PI)/4.0).GetValue();
	      
	      neighborAngle = atan2(vect1.GetY(), vect1.GetX());
	      
	      //argos::LOG<<"lowerPt ="<<lowerPt<<endl;
	      //argos::LOG<<"upperPt ="<<upperPt<<endl;
	      if(lowerPt < -3.1415){
			  lowerPt += 2*3.1415;
			  split = true;
			  //argos::LOG<<"*** lowerPt************************ ***" <<endl;
			  //the two ranges are [-3.1415, upperPt] and [lowerPt, 3.1415]
	      }
	      if(upperPt > 3.1415){
			  upperPt -= 2*3.1415;
			  split = true;
			  //argos::LOG<<"*** upperPt************************ ***" <<endl; 
			  //the two ranges are [-3.1415, upperPt] and [lowerPt, 3.1415]
	      }
	      
	      //argos::LOG<<it->first<< " robot position= " << it->second <<endl; 
		  //argos::LOG<<"vect1 ="<< vect1<< ", angle=" << neighborAngle << endl;
		    
		  vect2 = GetTarget() - GetPosition();
		  
		  targetAngle = atan2(vect2.GetY(), vect2.GetX());
		  
		  //argos::LOG<<"GetHeading().GetValue()-targetAngle = "<< fabs(GetHeading().GetValue()-targetAngle) <<endl;
		  if(vect1.SquareLength() < LoopFunctions->CameraRadiusSquared && fabs(GetHeading().GetValue()-targetAngle)<= 0.35){ // 0.35 = 20 degree
			//argos::LOG<<"GetTarget()="<<GetTarget()<< endl;
		    //argos::LOG<<"GetPosition()=" << GetPosition() <<endl;
		    //argos::LOG<<"vect2 = " << vect2 << endl;     
		  
		    if(split){
			  if( (neighborAngle >= -3.1415 && neighborAngle <= upperPt) || (neighborAngle >= lowerPt && neighborAngle <= 3.1415) ){
				 RobotDensity++;  
				 //argos::LOG<<"*** in the splitted ranges ***" <<endl;
			  }
			}
		    else if(neighborAngle >= lowerPt && neighborAngle <= upperPt) {
		      RobotDensity++;
		      //argos::LOG<<"*** in the range ***" <<endl;
		    }
		    //argos::LOG << controllerID<< " detects " << RobotDensity<< " robots."<<endl;
		  }
		
	  }
	}
	
}

/*****
 * If the robot has just picked up a food item, this function will be called
 * so that the food density in the local region is analyzed and saved. This
 * helps facilitate calculations for pheromone laying.
 *
 * Ideally, given that: [*] is food, and [=] is a robot
 *
 * [*] [*] [*] | The maximum resource density that should be calculated is
 * [*] [=] [*] | equal to 9, counting the food that the robot just picked up
 * [*] [*] [*] | and up to 8 of its neighbors.
 *
 * That being said, the random and non-grid nature of movement will not
 * produce the ideal result most of the time. This is especially true since
 * item detection is based on distance calculations with circles.
 *****/
void CPFA_controller::SetLocalResourceDensity() {
	argos::CVector2 distance;

	// remember: the food we picked up is removed from the foodList before this function call
	// therefore compensate here by counting that food (which we want to count)
	ResourceDensity = 1;

	/* Calculate resource density based on the global food list positions. */
	for(size_t i = 0; i < LoopFunctions->FoodList.size(); i++) {
	  distance = GetPosition() - LoopFunctions->FoodList[i];

	  if(distance.SquareLength() < LoopFunctions->SearchRadiusSquared*2) { //multiply 2 to use the diagonal distance
	    ResourceDensity++;
		LoopFunctions->FoodColoringList[i] = argos::CColor::ORANGE;
		LoopFunctions->ResourceDensityDelay = SimulationTick() + SimulationTicksPerSecond() * 10;
	  }
	}
 
	/* Set the fidelity position to the robot's current position. */
    SiteFidelityPosition = GetPosition();
    isUsingSiteFidelity = true;
    updateFidelity = true; 
    TrailToShare.push_back(SiteFidelityPosition);
    LoopFunctions->FidelityList[controllerID] = SiteFidelityPosition;
    /* Delay for 4 seconds (simulate iAnts scannning rotation). */
	//  Wait(4); // This function is broken. It causes the rover to move in the wrong direction after finishing its local resource density test 

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "(Survey): " << ResourceDensity << endl;
	//log_output_stream << "SiteFidelityPosition: " << SiteFidelityPosition << endl;
	//log_output_stream.close();
}

/*****
 * Update the global site fidelity list for graphics display and add a new fidelity position.
 *****/
void CPFA_controller::SetFidelityList(argos::CVector2 newFidelity) {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/*for(size_t i = 0; i < LoopFunctions->FidelityList.size(); i++) {
  if((LoopFunctions->FidelityList[i] - SiteFidelityPosition).SquareLength() != 0.0) {
			newFidelityList.push_back(LoopFunctions->FidelityList[i]);
		}
	} */


	/* Update the global fidelity list. */
	//LoopFunctions->FidelityList = newFidelityList;

        LoopFunctions->FidelityList[controllerID] = newFidelity;
	/* Add the robot's new fidelity position to the global fidelity list. */
	//LoopFunctions->FidelityList.push_back(newFidelity);
 

	/* Update the local fidelity position for this robot. */
	SiteFidelityPosition = newFidelity;
 
  updateFidelity = true;
}

/*****
 * Update the global site fidelity list for graphics display and remove the old fidelity position.
 *****/
void CPFA_controller::SetFidelityList() {
	std::vector<argos::CVector2> newFidelityList;

	/* Remove this robot's old fidelity position from the fidelity list. */
	/* Update the global fidelity list. */
        LoopFunctions->FidelityList.erase(controllerID);
 SiteFidelityPosition = CVector2(10000, 10000);
 updateFidelity = true; 
}

/*****
 * Update the pheromone list and set the target to a pheromone position.
 * return TRUE:  pheromone was successfully targeted
 *        FALSE: pheromones don't exist or are all inactive
 *****/
bool CPFA_controller::SetTargetPheromone() {
	argos::Real maxStrength = 0.0, randomWeight = 0.0;
	bool isPheromoneSet = false;

 if(LoopFunctions->PheromoneList.size()==0) return isPheromoneSet; //the case of no pheromone.
	/* update the pheromone list and remove inactive pheromones */

	/* default target = nest; in case we have 0 active pheromones */
	//SetIsHeadingToNest(true);
	//SetTarget(LoopFunctions->NestPosition);
	/* Calculate a maximum strength based on active pheromone weights. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		if(LoopFunctions->PheromoneList[i].IsActive()) {
			maxStrength += LoopFunctions->PheromoneList[i].GetWeight();
		}
	}

	/* Calculate a random weight. */
	randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, maxStrength));

	/* Randomly select an active pheromone to follow. */
	for(size_t i = 0; i < LoopFunctions->PheromoneList.size(); i++) {
		   if(randomWeight < LoopFunctions->PheromoneList[i].GetWeight()) {
			       /* We've chosen a pheromone! */
			       SetIsHeadingToNest(false);
          SetTarget(LoopFunctions->PheromoneList[i].GetLocation());
          TrailToFollow = LoopFunctions->PheromoneList[i].GetTrail();
          isPheromoneSet = true;
          /* If we pick a pheromone, break out of this loop. */
          break;
     }

     /* We didn't pick a pheromone! Remove its weight from randomWeight. */
     randomWeight -= LoopFunctions->PheromoneList[i].GetWeight();
	}

	//ofstream log_output_stream;
	//log_output_stream.open("cpfa_log.txt", ios::app);
	//log_output_stream << "Found: " << LoopFunctions->PheromoneList.size()  << " waypoints." << endl;
	//log_output_stream << "Follow waypoint?: " << isPheromoneSet << endl;
	//log_output_stream.close();

	return isPheromoneSet;
}

/*****
 * Calculate and return the exponential decay of "value."
 *****/
argos::Real CPFA_controller::GetExponentialDecay(argos::Real w, argos::Real time, argos::Real lambda) {
	/* convert time into units of haLoopFunctions-seconds from simulation frames */
	//time = time / (LoopFunctions->TicksPerSecond / 2.0);

	//LOG << "time: " << time << endl;
	//LOG << "correlation: " << (value * exp(-lambda * time)) << endl << endl;

	//return (value * std::exp(-lambda * time));
    Real     twoPi       = (CRadians::TWO_PI).GetValue();
    return w + (twoPi-w)* exp(-lambda * time);
}

/*****
 * Provides a bound on the value by rolling over a la modulo.
 *****/
argos::Real CPFA_controller::GetBound(argos::Real value, argos::Real min, argos::Real max) {
	/* Calculate an offset. */
	argos::Real offset = std::abs(min) + std::abs(max);

	/* Increment value by the offset while it's less than min. */
	while (value < min) {
			value += offset;
	}

	/* Decrement value by the offset while it's greater than max. */
	while (value > max) {
			value -= offset;
	}

	/* Return the bounded value. */
	return value;
}

size_t CPFA_controller::GetSearchingTime(){//qilu 10/22
    return searchingTime;
}
size_t CPFA_controller::GetTravelingTime(){//qilu 10/22
    return travelingTime;
}

string CPFA_controller::GetStatus(){//qilu 10/22
    //DEPARTING, SEARCHING, RETURNING
    if (CPFA_state == DEPARTING) return "DEPARTING";
    else if (CPFA_state ==SEARCHING)return "SEARCHING";
    else if (CPFA_state == RETURNING)return "RETURNING";
    else if (CPFA_state == SURVEYING) return "SURVEYING";
    //else if (MPFA_state == INACTIVE) return "INACTIVE";
    else return "SHUTDOWN";
    
}


/*****
 * Return the Poisson cumulative probability at a given k and lambda.
 *****/
argos::Real CPFA_controller::GetPoissonCDF(argos::Real k, argos::Real lambda) {
	argos::Real sumAccumulator       = 1.0;
	argos::Real factorialAccumulator = 1.0;

	for (size_t i = 1; i <= floor(k); i++) {
		factorialAccumulator *= i;
		sumAccumulator += pow(lambda, i) / factorialAccumulator;
	}

	return (exp(-lambda) * sumAccumulator);
}

void CPFA_controller::UpdateTargetRayList() {
	if(SimulationTick() % LoopFunctions->DrawDensityRate == 0 && LoopFunctions->DrawTargetRays == 1) {
		/* Get position values required to construct a new ray */
		argos::CVector2 t(GetTarget());
		argos::CVector2 p(GetPosition());
		argos::CVector3 position3d(p.GetX(), p.GetY(), 0.02);
		argos::CVector3 target3d(t.GetX(), t.GetY(), 0.02);

		/* scale the target ray to be <= searchStepSize */
		argos::Real length = std::abs(t.Length() - p.Length());

		if(length > SearchStepSize) {
			MyTrail.clear();
		} else {
			/* add the ray to the robot's target trail */
			argos::CRay3 targetRay(target3d, position3d);
			MyTrail.push_back(targetRay);

			/* delete the oldest ray from the trail */
			if(MyTrail.size() > MaxTrailSize) {
				MyTrail.erase(MyTrail.begin());
			}

			LoopFunctions->TargetRayList.insert(LoopFunctions->TargetRayList.end(), MyTrail.begin(), MyTrail.end());
			// loopFunctions.TargetRayList.push_back(myTrail);
		}
	}
}

REGISTER_CONTROLLER(CPFA_controller, "CPFA_controller")
