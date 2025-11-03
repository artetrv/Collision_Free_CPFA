#include "CPFA_controller.h"
#include <unistd.h>

CPFA_controller::CPFA_controller() :
	RNG(argos::CRandom::CreateRNG("argos")),
	isInformed(false),
	isHoldingFood(false),
	isUsingSiteFidelity(false),
	isGivingUpSearch(false),
	isFollowingRandomTarget(false),
	randomTargetSearchTime(0),
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
        lastMemoryStorageTime(0.0),
        VisitCountThreshold(3),
        isRecordingTrajectory(false)
{
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
     //print state of the robot
	//  argos::LOG << "Robot " << controllerID << " state: " << GetStatus() << std::endl;
	//print the current target only when it is changed
	// if(GetTarget() != previousTarget) {
	// 	previousTarget = GetTarget();
	// 	argos::LOG << "Robot " << controllerID << " new target: " << GetTarget() << std::endl;
	// }
	//print out state of robot
	// argos::LOG << "Robot " << controllerID << " state: " << GetStatus() << std::endl;

	// Periodic location storage every 5 seconds - only during random search and only in enhanced mode
	// Robot memory logging begins only after spiral search is complete
	if(
	   curr_time_in_seconds - lastMemoryStorageTime >= 5.0 && 
	   !isHoldingFood && 
	   !isInformed && 
	   CPFA_state == SEARCHING &&
	   !isUsingSpiralSearch
	) {
		argos::CVector2 currentPosition = GetPosition();
		
		// Add current position to robotMemory
		robotMemory.push_back(currentPosition);
		// argos::LOG << "Robot " << controllerID << " (Enhanced) storing position: " << currentPosition << std::endl;
		// Maintain sliding window of maximum 5 locations
		if(robotMemory.size() > 5) {
			robotMemory.erase(robotMemory.begin()); // Remove the oldest entry
		}
		
		// Update the last storage time
		lastMemoryStorageTime = curr_time_in_seconds;
	}
	if(curr_time_in_seconds - last_time_in_seconds >= 0)
	{
		// CVector2 position2d(GetPosition().GetX(), GetPosition().GetY());
		
		// CVector3 position3d(GetPosition().GetX(), GetPosition().GetY(), 0.00);
		// CVector3 target3d(previous_position.GetX(), previous_position.GetY(), 0.00);
		// CRay3 targetRay(target3d, position3d);
		// myTrail.push_back(targetRay);
		// LoopFunctions->Trajectory[controllerID].push_back(position2d);
		// //since it costs a lot of memeory, I commented it. qilu 06/2023. You can uncomment it if you want to show the trails.
		// LoopFunctions->TargetRayList.push_back(targetRay);
		// LoopFunctions->TargetRayColorList.push_back(TrailColor);
		// //argos::LOG<< "TargetRayList size =" << LoopFunctions->TargetRayList.size() <<endl;
		// previous_position = GetPosition();
		// last_time_in_seconds = curr_time_in_seconds;
		
		// // Record position for trajectory if tracking is active
		// if(isRecordingTrajectory) {
		// 	currentTrajectory.push_back(GetPosition());
		// }
		
		// last_time_in_seconds = curr_time_in_seconds;
     }
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
    
    // Reset robot memory tracking
    robotMemory.clear();
    lastMemoryStorageTime = 0.0;
    
    // Reset trajectory tracking
    currentTrajectory.clear();
    isRecordingTrajectory = false;
    
    // Reset spiral search locations
    spiralSearchLocations.clear();
    visitedSpiralLocations.clear();
    currentSpiralIndex = 0;
    isUsingSpiralSearch = false;
    
    // Reset random target tracking
    isFollowingRandomTarget = false;
    randomTargetSearchTime = 0;
    
  	LoopFunctions->CollisionTime=0; //qilu 09/26/2016
    
    
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
	}
}

bool CPFA_controller::IsInTheNest() {
    
	return ((GetPosition() - LoopFunctions->NestPosition).SquareLength()
		< LoopFunctions->NestRadiusSquared);
	}

void CPFA_controller::SetLoopFunctions(CPFA_loop_functions* lf) {
	LoopFunctions = lf;
	
	// Initialize SearchAlgorithmMode from loop functions
	SearchAlgorithmMode = LoopFunctions->getSearchAlgorithmMode();

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

void CPFA_controller::Departing()
{
     //LOG<<"Departing..."<<endl;
    argos::Real distanceToTarget = (GetPosition() - GetTarget()).Length();
    argos::Real randomNumber = RNG->Uniform(argos::CRange<argos::Real>(0.0, 1.0));

    /*
    ofstream log_output_stream;
    log_output_stream.open("cpfa_log.txt", ios::app);
    log_output_stream << "Distance to target: " << distanceToTarget << endl;
    log_output_stream << "Current Position: " << GetPosition() << ", Target: " << GetTarget() << endl;
    log_output_stream.close();
    */

	/* When not informed, continue to travel until randomly switching to the searching state. */
    if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
       if(isInformed == false){
           
		       // Check if target is near a wall and use more lenient tolerance
       argos::CVector2 target = GetTarget();
       argos::Real wallBuffer = 0.25; // Distance to consider "near wall"
       bool nearWall = (target.GetX() > ForageRangeX.GetMax() - wallBuffer || 
                       target.GetX() < ForageRangeX.GetMin() + wallBuffer ||
                       target.GetY() > ForageRangeY.GetMax() - wallBuffer || 
                       target.GetY() < ForageRangeY.GetMin() + wallBuffer);
       
       argos::Real tolerance = nearWall ? TargetDistanceTolerance * 4.0 : TargetDistanceTolerance;
    //    if(SimulationTick()%(5*SimulationTicksPerSecond())==0 && randomNumber < LoopFunctions->ProbabilityOfSwitchingToSearching){
	       if(distanceToTarget < tolerance){		
			 //LOG<<"Switch to search..."<<endl;
                 Stop();
                 SearchTime = 0;
                 CPFA_state = SEARCHING;
                 travelingTime+=SimulationTick()-startTime;//qilu 10/22
                 startTime = SimulationTick();//qilu 10/22
            
                 argos::Real USV = LoopFunctions->UninformedSearchVariation.GetValue();
                 argos::Real rand = RNG->Gaussian(USV);
                 argos::CRadians rotation(rand);
                 argos::CRadians angle1(rotation.UnsignedNormalize());
                 argos::CRadians angle2(GetHeading().UnsignedNormalize());
                 argos::CRadians turn_angle(angle1 + angle2);
                 argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                 SetIsHeadingToNest(false);
                 SetTarget(turn_vector + GetPosition());
				//  argos::LOG << "Robot " << controllerID << " is switching to SEARCHING state" << std::endl;
		   }
		//    else if(distanceToTarget < tolerance){
		// 	 SetRandomSearchLocation();
		// 	//  argos::LOG << "Reached target, setting new random search location: " << GetTarget() << " at tick: " << SimulationTick() << std::endl;
		//    }
	   }
	 } 
		 
     /* Are we informed? I.E. using site fidelity or pheromones. */	
     if(isInformed && distanceToTarget < TargetDistanceTolerance) {
          SearchTime = 0;
          CPFA_state = SEARCHING;
          travelingTime+=SimulationTick()-startTime;//qilu 10/22
          startTime = SimulationTick();//qilu 10/22

          if(isUsingSiteFidelity) {
               isUsingSiteFidelity = false;
               SetFidelityList();
          }
     }
     else{ // based on the density of robots, decide to do random search
		 if(isInformed == true && SimulationTick()% SimulationTicksPerSecond() ==0 ){
			 //LOG<<"Departing..."<<endl;
			
	      }
	 }


}

void CPFA_controller::Searching() {
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
       
       // Check if target is near a wall and use more lenient tolerance
       argos::CVector2 target = GetTarget();
       argos::Real wallBuffer = 0.25; // Distance to consider "near wall"
       bool nearWall = (target.GetX() > ForageRangeX.GetMax() - wallBuffer || 
                       target.GetX() < ForageRangeX.GetMin() + wallBuffer ||
                       target.GetY() > ForageRangeY.GetMax() - wallBuffer || 
                       target.GetY() < ForageRangeY.GetMin() + wallBuffer);
       
       argos::Real tolerance = nearWall ? TargetDistanceTolerance * 4.0 : TargetDistanceTolerance;
       
	if((!nearWall && IsAtTarget()) || (nearWall && distance.Length() < tolerance)) {
         // randomly give up searching
         if(SimulationTick()% (5*SimulationTicksPerSecond())==0 && random < LoopFunctions->ProbabilityOfReturningToNest && !isUsingSpiralSearch) {
             
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

             // Reset spiral search when giving up
             isUsingSpiralSearch = false;
             currentSpiralIndex = 0;

             // Stop trajectory recording and export if we were recording
             if(isRecordingTrajectory) {
                 LoopFunctions->exportRandomSearchTrajectory(controllerID, currentTrajectory, targetFromRandomSearch);
                 currentTrajectory.clear();
                 isRecordingTrajectory = false;
             }

             /*
             ofstream log_output_stream;
             log_output_stream.open("giveup.txt", ios::app);
             log_output_stream << "Give up: " << SimulationTick() / SimulationTicksPerSecond() << endl;
             log_output_stream.close();
             */
     
             return; 
             
         }
         argos::Real USCV = LoopFunctions->UninformedSearchVariation.GetValue();
         argos::Real rand = RNG->Gaussian(USCV);

         // uninformed search
         if(isInformed == false) {
          // Check if we have spiral locations to explore
          if(isUsingSpiralSearch && !spiralSearchLocations.empty()) {
				// argos::LOG << "Robot " << controllerID << " reached spiral location: " << GetTarget() << std::endl;
				
				// Store the visited spiral location to be sent when returning to nest

				//skip if its the first iteration. im not sure why but it always add the first location twice so we skip the first time to avoid false visits.
				if(currentSpiralIndex > 0) {
					visitedSpiralLocations.push_back(GetTarget());
				}
              
              // If we've visited all spiral locations, disable spiral search and continue with normal random search
              if(currentSpiralIndex >= spiralSearchLocations.size()) {
                  isUsingSpiralSearch = false;
                  currentSpiralIndex = 0;
                  
                  // Generate normal random search movement
                  argos::CRadians rotation(rand);
                  argos::CRadians angle1(rotation);
                  argos::CRadians angle2(GetHeading());
                  argos::CRadians turn_angle(angle1 + angle2);
                  argos::CVector2 turn_vector(SearchStepSize, turn_angle);
                  SetIsHeadingToNest(false);
                  SetTarget(turn_vector + GetPosition());
              } else {
                  // Set target to the next spiral location
                  SetIsHeadingToNest(false);
                  SetTarget(spiralSearchLocations[currentSpiralIndex]);
                //   argos::LOG << "Robot " << controllerID << " moving to spiral location " << currentSpiralIndex << ": " << spiralSearchLocations[currentSpiralIndex] << std::endl;
				  // Move to the next spiral location
				  currentSpiralIndex++;
              }
          } else {
              // Normal uninformed search behavior
              argos::CRadians rotation(rand);
              argos::CRadians angle1(rotation);
              argos::CRadians angle2(GetHeading());
              argos::CRadians turn_angle(angle1 + angle2);
              argos::CVector2 turn_vector(SearchStepSize, turn_angle);
          
              //argos::LOG << "UNINFORMED SEARCH: rotation: " << angle1 << std::endl;
              //argos::LOG << "UNINFORMED SEARCH: old heading: " << angle2 << std::endl;
          
              /*
              ofstream log_output_stream;
              log_output_stream.open("uninformed_angle1.log", ios::app);
              log_output_stream << angle1.GetValue() << endl;
              log_output_stream.close();
          
              log_output_stream.open("uninformed_angle2.log", ios::app);
              log_output_stream << angle2.GetValue() << endl;
              log_output_stream.close();
          
              log_output_stream.open("uninformed_turning_angle.log", ios::app);
              log_output_stream << turn_angle.GetValue() << endl;
              log_output_stream.close();
              */
              SetIsHeadingToNest(false);
              SetTarget(turn_vector + GetPosition());
          }
         }
         // informed search
         else{
          
              SetIsHeadingToNest(false);
            //   argos::LOG << "Robot: " << GetId() << " - INFORMED SEARCH: Reached target location. " << std::endl;
            //   if(IsAtTarget()) {
                  size_t          t           = SearchTime++;
                  argos::Real     twoPi       = (argos::CRadians::TWO_PI).GetValue();
                  argos::Real     pi          = (argos::CRadians::PI).GetValue();
                  argos::Real     isd         = LoopFunctions->RateOfInformedSearchDecay;
	                  /*argos::Real     correlation = GetExponentialDecay((2.0 * twoPi) - LoopFunctions->UninformedSearchVariation.GetValue(), t, isd);
	                  argos::Real     rand = RNG->Gaussian(correlation + LoopFunctions->UninformedSearchVariation.GetValue());
	                       */ //qilu 09/24/2016
	                  Real correlation = GetExponentialDecay(rand, t, isd);
	                  //argos::CRadians rotation(GetBound(rand, -pi, pi));
	                  argos::CRadians rotation(GetBound(correlation, -pi, pi));//qilu 09/24/2016
                  argos::CRadians angle1(rotation);
                  argos::CRadians angle2(GetHeading());
                  argos::CRadians turn_angle(angle2 + angle1);
                  argos::CVector2 turn_vector(SearchStepSize, turn_angle);
          
                  //argos::LOG << "INFORMED SEARCH: rotation: " << angle1 << std::endl;
                  //argos::LOG << "INFORMED SEARCH: old heading: " << angle2 << std::endl;
          
                  /*
                  ofstream log_output_stream;
                  log_output_stream.open("informed_angle1.log", ios::app);
                  log_output_stream << angle1.GetValue() << endl;
                  log_output_stream.close();
          
                  log_output_stream.open("informed_angle2.log", ios::app);
                  log_output_stream << angle2.GetValue() << endl;
                  log_output_stream.close();
          
                  log_output_stream.open("informed_turning_angle.log", ios::app);
                  log_output_stream << turn_angle.GetValue() << endl;
                  log_output_stream.close();
                  */
                  SetTarget(turn_vector + GetPosition());
            //   }
         }
	  } //not reach the target location
	  else {
			 //argos::LOG << "SEARCH: Haven't reached destination. " << GetPosition() << "," << GetTarget() << std::endl;
		
			 
	  }
    }
	else {
		   //argos::LOG << "SEARCH: Carrying food." << std::endl;
	}

	// Food has been found, change state to RETURNING and go to the nest
	//else {
	//	SetTarget(LoopFunctions->NestPosition);
	//	CPFA_state = RETURNING;
	//}
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
		SetIsHeadingToNest(false); // Turn on error for this
		SetTarget(LoopFunctions->NestPosition); 
		CPFA_state = RETURNING;
		survey_count = 0; // Reset
        searchingTime+=SimulationTick()-startTime;//qilu 10/22
        startTime = SimulationTick();//qilu 10/22
        
        // Stop trajectory recording and export if we were recording
        if(isRecordingTrajectory) {
            LoopFunctions->exportRandomSearchTrajectory(controllerID, currentTrajectory, targetFromRandomSearch);
            currentTrajectory.clear();
            isRecordingTrajectory = false;
        }
            
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
	// "scan" for food only every half of a second
	if((SimulationTick() % (SimulationTicksPerSecond() / 2)) == 0) {
		SetHoldingFood();
	}
	// Are we there yet? (To the nest, that is.)
	if(IsInTheNest()) {

		/* LOGIC TO SEND LAST 5 LOCATIONS TO CENTRAL CONTROLLER */
		// if(SearchAlgorithmMode == 1 && !robotMemory.empty()) {
		if(!robotMemory.empty()) {
			// argos::LOG << "Robot " << controllerID << " returning to nest with " 
			// 		   << robotMemory.size() << " stored locations:" << std::endl;
			
			// for(size_t i = 0; i < robotMemory.size(); i++) {
			// 	argos::LOG << "  Location " << (i+1) << ": " << robotMemory[i] << std::endl;
			// }
			
			// Send locations to the central controller to update the grid
			LoopFunctions->receiveRobotMemory(controllerID, robotMemory);
			
			robotMemory.clear();
		}

		// Send visited spiral locations to update the grid map
		if(!visitedSpiralLocations.empty()) {
			// argos::LOG << "Robot " << controllerID << " returning to nest with " 
			// 		   << visitedSpiralLocations.size() << " visited spiral locations" << std::endl;
			
			// Send spiral locations to the central controller to update the grid
			LoopFunctions->receiveRobotMemory(controllerID, visitedSpiralLocations);
			
			visitedSpiralLocations.clear();
		}

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
		        // Turn off red LEDs when switching to site fidelity
		        m_pcLEDs->SetAllColors(CColor::GREEN);
	    }
      // use pheromone waypoints
      else if(SetTargetPheromone()) {
          //log_output_stream << "Using site pheremone" << endl;
          isInformed = true;
          isUsingSiteFidelity = false;
          // Turn off red LEDs when switching to pheromone following
          m_pcLEDs->SetAllColors(CColor::GREEN);
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

// This setrandomsearchlocation function is set to randomly sample 5 cells and probabilistically select one based on visit counts
void CPFA_controller::SetRandomSearchLocation() {
    // Set LEDs to red to indicate random search mode
    m_pcLEDs->SetAllColors(CColor::RED);
    
    argos::Real x = 0.0, y = 0.0;
    argos::CVector2 candidateTarget;
    
    // Apply enhanced algorithm with sampling if mode is 1 (enhanced)
    if(SearchAlgorithmMode == 1) {
        const int numSamples = 5;
        std::vector<argos::CVector2> candidates(numSamples);
        std::vector<int> visitCounts(numSamples);
        
        // Generate 5 random candidate locations
        for(int i = 0; i < numSamples; i++) {
            x = RNG->Uniform(ForageRangeX);
            y = RNG->Uniform(ForageRangeY);
            candidates[i] = argos::CVector2(x, y);
            
            // OLD LOGIC: Calculate visit count for individual cell only
            // visitCounts[i] = LoopFunctions->getGridVisitCount(candidates[i]);
            
            // NEW LOGIC: Calculate visit count for area (cell + immediate neighbors)
            int areaVisitCount = 0;
            argos::Real cellSize = 0.25; // 0.75 meters grid cell size (3x3 of original 0.25m cells)
            
            // Check the 8 immediate neighbors plus the center cell (3x3 area)
            for(int dx = -1; dx <= 1; dx++) {
                for(int dy = -1; dy <= 1; dy++) {
                    argos::CVector2 neighborPos = candidates[i] + argos::CVector2(dx * cellSize, dy * cellSize);
                    areaVisitCount += LoopFunctions->getGridVisitCount(neighborPos);
                }
            }
            
            visitCounts[i] = areaVisitCount;
        }
        
        // Find cells with visit count of 0
        std::vector<int> zeroIndices;
        for(int i = 0; i < numSamples; i++) {
            if(visitCounts[i] == 0) {
                zeroIndices.push_back(i);
            }
        }
        
        int selectedIndex = 0;
        
        if(!zeroIndices.empty()) {
            // If we have zero visit count cells, select randomly among them
            int randomZeroIndex = RNG->Uniform(argos::CRange<argos::UInt32>(0, zeroIndices.size()));
            selectedIndex = zeroIndices[randomZeroIndex];
            // argos::LOG << "Robot " << controllerID << " selected unvisited area from " << zeroIndices.size() << " unvisited area candidates" << std::endl;
        } else {
            // No zero visit counts, use weighted selection based on 1/visitCount
            std::vector<argos::Real> weights(numSamples);
            argos::Real totalWeight = 0.0;
            
            // Calculate weights (1/visitCount)
            for(int i = 0; i < numSamples; i++) {
                weights[i] = 1.0 / visitCounts[i];
                totalWeight += weights[i];
            }
            
            // Randomly select based on weights
            argos::Real randomWeight = RNG->Uniform(argos::CRange<argos::Real>(0.0, totalWeight));
            argos::Real cumulativeWeight = 0.0;
            
            for(int i = 0; i < numSamples; i++) {
                cumulativeWeight += weights[i];
                if(randomWeight <= cumulativeWeight) {
                    selectedIndex = i;
                    break;
                }
            }
            
            argos::LOG << "Robot " << controllerID << " selected location using weighted selection with area visit count " << visitCounts[selectedIndex] << std::endl;
        }
        
        candidateTarget = candidates[selectedIndex];
    } else {
        // For SearchAlgorithmMode == 0 (baseline), use simple random generation
        x = RNG->Uniform(ForageRangeX);
        y = RNG->Uniform(ForageRangeY);
        candidateTarget = argos::CVector2(x, y);
    }
        
    // Generate spiral search locations only for enhanced mode (SearchAlgorithmMode == 1)
    spiralSearchLocations.clear();
    currentSpiralIndex = 0;
    isUsingSpiralSearch = false;
    
    if(SearchAlgorithmMode == 1) {
        argos::Real cellSize = 0.25; // 0.75 meters (3x3 of original 0.25m cells)
        
        // Extended spiral pattern: center, right, up-right, up, up-left, left, down-left, down
        spiralSearchLocations.push_back(candidateTarget); // Location 0: center (original target)
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(cellSize, 0.0)); // Location 1: right
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(cellSize, cellSize)); // Location 2: up-right
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(0.0, cellSize)); // Location 3: up
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-cellSize, cellSize)); // Location 4: up-left
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-cellSize, 0.0)); // Location 5: left
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-cellSize, -cellSize)); // Location 6: down-left
        spiralSearchLocations.push_back(candidateTarget + argos::CVector2(0.0, -cellSize)); // Location 7: down
        
        // Initialize spiral search tracking for enhanced mode only
        isUsingSpiralSearch = true;
    }
    
    SetIsHeadingToNest(true); // Turn off error for this
    SetTarget(candidateTarget);
    targetFromRandomSearch = candidateTarget;
    
    // Set flag to indicate we're following a random target
    isFollowingRandomTarget = true;
    randomTargetSearchTime = 0;
    
    // Start trajectory recording
    currentTrajectory.clear();
    isRecordingTrajectory = true;
    currentTrajectory.push_back(GetPosition()); // Record starting position
    
    // argos::LOG << "Robot " << controllerID << " setting random search target: " << candidateTarget << std::endl;
}

// This setrandomsearchlocation uses full grid scanning to find the least visited cell
// void CPFA_controller::SetRandomSearchLocation() {
//     m_pcLEDs->SetAllColors(CColor::RED);
    
//     argos::Real x = 0.0, y = 0.0;
//     argos::CVector2 candidateTarget;
    
//     // Apply enhanced algorithm with full grid scanning if mode is 1 (enhanced)
//     if(SearchAlgorithmMode == 1) {
//         // Get grid parameters dynamically from loop functions
//         const size_t gridWidth = LoopFunctions->GridWidth;
//         const size_t gridHeight = LoopFunctions->GridHeight;
//         const argos::Real arenaWidth = ForageRangeX.GetMax() - ForageRangeX.GetMin();
//         const argos::Real arenaHeight = ForageRangeY.GetMax() - ForageRangeY.GetMin();
//         const argos::Real cellSizeX = arenaWidth / gridWidth;
//         const argos::Real cellSizeY = arenaHeight / gridHeight;
        
//         std::vector<argos::CVector2> minVisitCells;
//         int minVisitCount = INT_MAX;
        
//         // Scan entire grid to find cells with minimum visit count
//         // This is O(gridWidth × gridHeight)
//         for(size_t i = 0; i < gridWidth; i++) {
//             for(size_t j = 0; j < gridHeight; j++) {
//                 // Convert grid indices to world coordinates (center of each cell)
//                 argos::Real worldX = ForageRangeX.GetMin() + (i + 0.5) * cellSizeX;
//                 argos::Real worldY = ForageRangeY.GetMin() + (j + 0.5) * cellSizeY;
//                 argos::CVector2 cellCenter(worldX, worldY);
                
//                 // Get visit count for this cell
//                 int visitCount = LoopFunctions->getGridVisitCount(cellCenter);
                
//                 if(visitCount < minVisitCount) {
//                     // Found new minimum - clear list and add this cell
//                     minVisitCount = visitCount;
//                     minVisitCells.clear();
//                     minVisitCells.push_back(cellCenter);
//                 } else if(visitCount == minVisitCount) {
//                     // Found another cell with same minimum count - add to list
//                     minVisitCells.push_back(cellCenter);
//                 }
//             }
//         }
        
//         // Randomly select from cells with minimum visit count
//         if(!minVisitCells.empty()) {
//             int randomIndex = RNG->Uniform(argos::CRange<argos::UInt32>(0, minVisitCells.size()));
//             candidateTarget = minVisitCells[randomIndex];
            
//             argos::LOG << "Robot " << controllerID << " selected cell with minimum visit count " 
//                       << minVisitCount << " from " << minVisitCells.size() 
//                       << " equally minimal cells" << std::endl;
//         } else {
//             // Fallback to random selection (shouldn't happen)
//             x = RNG->Uniform(ForageRangeX);
//             y = RNG->Uniform(ForageRangeY);
//             candidateTarget = argos::CVector2(x, y);
//         }
        
//         // Generate spiral search locations for enhanced mode
//         spiralSearchLocations.clear();
//         currentSpiralIndex = 0;
//         isUsingSpiralSearch = true;
        
//         // Use average cell size for spiral pattern (in case grid is not square)
//         // argos::Real avgCellSize = (cellSizeX + cellSizeY) / 2.0;
//         argos::Real avgCellSize = 0.25;
//         // Extended spiral pattern: center, right, up-right, up, up-left, left, down-left, down
//         spiralSearchLocations.push_back(candidateTarget); // Location 0: center (original target)
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(avgCellSize, 0.0)); // Location 1: right
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(avgCellSize, avgCellSize)); // Location 2: up-right
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(0.0, avgCellSize)); // Location 3: up
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-avgCellSize, avgCellSize)); // Location 4: up-left
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-avgCellSize, 0.0)); // Location 5: left
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(-avgCellSize, -avgCellSize)); // Location 6: down-left
//         spiralSearchLocations.push_back(candidateTarget + argos::CVector2(0.0, -avgCellSize)); // Location 7: down
        
//     } else {
//         // For SearchAlgorithmMode == 0 (baseline), use simple random generation
//         spiralSearchLocations.clear();
//         currentSpiralIndex = 0;
//         isUsingSpiralSearch = false;
        
//         x = RNG->Uniform(ForageRangeX);
//         y = RNG->Uniform(ForageRangeY);
//         candidateTarget = argos::CVector2(x, y);
//     }
    
//     SetIsHeadingToNest(true); // Turn off error for this
//     SetTarget(candidateTarget);
//     targetFromRandomSearch = candidateTarget;
    
//     // Set flag to indicate we're following a random target
//     isFollowingRandomTarget = true;
//     randomTargetSearchTime = 0;
    
//     // Start trajectory recording
//     currentTrajectory.clear();
//     isRecordingTrajectory = true;
//     currentTrajectory.push_back(GetPosition()); // Record starting position
    
//     // argos::LOG << "Robot " << controllerID << " setting random search target: " << candidateTarget << std::endl;
// }

/*****
 * Check if the iAnt is finding food. This is defined as the iAnt being within
 * the distance tolerance of the position of a food item. If the iAnt has found
 * food then the appropriate boolean flags are triggered.
 *****/
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
					 j = i + 1;
					 searchingTime+=SimulationTick()-startTime;
					 startTime = SimulationTick();
					 
					 // Reset spiral search when food is found
					 isUsingSpiralSearch = false;
					 currentSpiralIndex = 0;
					 
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
