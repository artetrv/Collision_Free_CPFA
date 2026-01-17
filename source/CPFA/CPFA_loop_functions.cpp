#include "CPFA_loop_functions.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <cstring>
#include <iomanip>
#include <fstream>

CPFA_loop_functions::CPFA_loop_functions() :
	RNG(argos::CRandom::CreateRNG("argos")),
        SimTime(0),
	//MaxSimTime(3600 * GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()),
    MaxSimTime(0),//qilu 02/05/2021
        CollisionTime(0), 
        lastNumCollectedFood(0),
        currNumCollectedFood(0),
	ResourceDensityDelay(0),
	RandomSeed(GetSimulator().GetRandomSeed()),
	SimCounter(0),
	MaxSimCounter(1),
	VariableFoodPlacement(0),
	OutputData(0),
	DrawDensityRate(4),
	DrawIDs(1),
	DrawTrails(1),
	DrawTargetRays(1),
	FoodDistribution(2),
	FoodItemCount(256),
	PowerlawFoodUnitCount(256),
	NumberOfClusters(4),
	ClusterWidthX(8),
	ClusterWidthY(8),
	PowerRank(4),
	ProbabilityOfSwitchingToSearching(0.0),
	ProbabilityOfReturningToNest(0.0),
	UninformedSearchVariation(0.0),
	RateOfInformedSearchDecay(0.0),
	RateOfSiteFidelity(0.0),
	RateOfLayingPheromone(0.0),
	RateOfPheromoneDecay(0.0),
	SearchAlgorithmMode(0),  // Default to enhanced algorithm
	FoodRadius(0.05),
	FoodRadiusSquared(0.0025),
	NestRadius(0.12),
	NestRadiusSquared(0.0625),
	NestElevation(0.01),
	// We are looking at a 4 by 4 square (3 targets + 2*1/2 target gaps)
	SearchRadiusSquared((4.0 * FoodRadius) * (4.0 * FoodRadius)),
	CameraRadiusSquared(2.25),
	NumDistributedFood(0),
	FoodTarget88Percent(0), // Will be calculated after food distribution
	score(0),
	PrintFinalScore(0),
	RejectedLocationCounter(0),
	lastMilestone(0)
{
	// Initialize milestone tracking
	resourceCollectionMilestones.clear();
}

void CPFA_loop_functions::Init(argos::TConfigurationNode &node) {	
	
	// Clear any existing data from previous runs (except milestone data - that's handled by batch script)
	clearHeatmapData();
	clearDotplotData();
	clearTrajectoryData();
	clearFoodData();
 
	argos::CDegrees USV_InDegrees;
	argos::TConfigurationNode CPFA_node = argos::GetNode(node, "CPFA");

	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfSwitchingToSearching", ProbabilityOfSwitchingToSearching);
	argos::GetNodeAttribute(CPFA_node, "ProbabilityOfReturningToNest",      ProbabilityOfReturningToNest);
	argos::GetNodeAttribute(CPFA_node, "UninformedSearchVariation",         USV_InDegrees);
	argos::GetNodeAttribute(CPFA_node, "RateOfInformedSearchDecay",         RateOfInformedSearchDecay);
	argos::GetNodeAttribute(CPFA_node, "RateOfSiteFidelity",                RateOfSiteFidelity);
	argos::GetNodeAttribute(CPFA_node, "RateOfLayingPheromone",             RateOfLayingPheromone);
	argos::GetNodeAttribute(CPFA_node, "RateOfPheromoneDecay",              RateOfPheromoneDecay);
	argos::GetNodeAttribute(CPFA_node, "SearchAlgorithmMode",               SearchAlgorithmMode);
	
	argos::GetNodeAttribute(CPFA_node, "PrintFinalScore",                   PrintFinalScore);

	UninformedSearchVariation = ToRadians(USV_InDegrees);
	argos::TConfigurationNode settings_node = argos::GetNode(node, "settings");
	
	argos::GetNodeAttribute(settings_node, "MaxSimTimeInSeconds", MaxSimTime);

	MaxSimTime *= GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/05/2021 dyn2d error

	argos::GetNodeAttribute(settings_node, "MaxSimCounter", MaxSimCounter);
	argos::GetNodeAttribute(settings_node, "VariableFoodPlacement", VariableFoodPlacement);
	argos::GetNodeAttribute(settings_node, "OutputData", OutputData);
	argos::GetNodeAttribute(settings_node, "DrawIDs", DrawIDs);
	argos::GetNodeAttribute(settings_node, "DrawTrails", DrawTrails);
	argos::GetNodeAttribute(settings_node, "DrawTargetRays", DrawTargetRays);
	argos::GetNodeAttribute(settings_node, "FoodDistribution", FoodDistribution);
	argos::GetNodeAttribute(settings_node, "FoodItemCount", FoodItemCount);
	argos::GetNodeAttribute(settings_node, "PowerlawFoodUnitCount", PowerlawFoodUnitCount);
	argos::GetNodeAttribute(settings_node, "NumberOfClusters", NumberOfClusters);
	argos::GetNodeAttribute(settings_node, "ClusterWidthX", ClusterWidthX);
	argos::GetNodeAttribute(settings_node, "ClusterWidthY", ClusterWidthY);
	argos::GetNodeAttribute(settings_node, "FoodRadius", FoodRadius);
    argos::GetNodeAttribute(settings_node, "NestRadius", NestRadius);
	argos::GetNodeAttribute(settings_node, "NestElevation", NestElevation);
    argos::GetNodeAttribute(settings_node, "NestPosition", NestPosition);
    FoodRadiusSquared = FoodRadius*FoodRadius;

    //Number of distributed foods
    if (FoodDistribution == 1){
        NumDistributedFood = ClusterWidthX*ClusterWidthY*NumberOfClusters;
    }
    else{
        NumDistributedFood = FoodItemCount;  
    }
    

	// calculate the forage range and compensate for the robot's radius of 0.085m
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real rangeX = (ArenaSize.GetX() / 2.0) - 0.085;
	argos::Real rangeY = (ArenaSize.GetY() / 2.0) - 0.085;
	ForageRangeX.Set(-rangeX, rangeX);
	ForageRangeY.Set(-rangeY, rangeY);

        ArenaWidth = ArenaSize[0];
        
        // Create the grid with a default cell size of 1 meters
        create_grid(1.0);
        
       /* if(abs(NestPosition.GetX()) < -1) //quad arena
        {
            NestRadius *= sqrt(1 + log(ArenaWidth)/log(2));
        }
        else
        {
            NestRadius *= sqrt(log(ArenaWidth)/log(2));
        } */
        
        //argos::LOG<<"NestRadius="<<NestRadius<<endl;
	   // Send a pointer to this loop functions object to each controller.
	   argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	   argos::CSpace::TMapPerType::iterator it;
    
    Num_robots = footbots.size();
    argos::LOG<<"Number of robots="<<Num_robots<<endl;
	   for(it = footbots.begin(); it != footbots.end(); it++) {
   	   	argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
		      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        c2.SetLoopFunctions(this);
	    }
     
     
   NestRadiusSquared = NestRadius*NestRadius;
	
    SetFoodDistribution();
    
    // Calculate 88% of total food items for simulation completion
    FoodTarget88Percent = static_cast<size_t>(std::ceil(FoodList.size() * 1.0));
    // argos::LOG << "Food distribution set with " << FoodList.size() << " total food items" << std::endl;
    // argos::LOG << "Simulation will finish when " << FoodTarget88Percent << " food items (88%) are collected" << std::endl;
  
 ForageList.clear(); 
 last_time_in_minutes=0;
 
}


void CPFA_loop_functions::Reset() {
	   if(VariableFoodPlacement == 0) {
		      RNG->Reset();
	   }

    GetSpace().Reset();
    GetSpace().GetFloorEntity().Reset();
    MaxSimCounter = SimCounter;
    SimCounter = 0;
    score = 0;
   
    FoodList.clear();
    CollectedFoodList.clear();
    FoodColoringList.clear();
	PheromoneList.clear();
	FidelityList.clear();
    TargetRayList.clear();
    Trajectory.clear();
    
    SetFoodDistribution();
    
    // Recalculate 88% threshold after reset
    FoodTarget88Percent = static_cast<size_t>(std::ceil(FoodList.size() * 1.00));
    // argos::LOG << "Reset: Food distribution set with " << FoodList.size() << " total food items" << std::endl;
    // argos::LOG << "Reset: Simulation will finish when " << FoodTarget88Percent << " food items (88%) are collected" << std::endl;
    
    // Reset milestone tracking
    resourceCollectionMilestones.clear();
    lastMilestone = 0;
    
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    argos::CSpace::TMapPerType::iterator it;
   
    for(it = footbots.begin(); it != footbots.end(); it++) {
        argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
        BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
        CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
        MoveEntity(footBot.GetEmbodiedEntity(), c2.GetStartPosition(), argos::CQuaternion(), false);
    c2.Reset();
    }
}

void CPFA_loop_functions::PreStep() {
    SimTime++;
    curr_time_in_minutes = getSimTimeInSeconds()/60.0;
    if(curr_time_in_minutes - last_time_in_minutes==1){
		      
        ForageList.push_back(currNumCollectedFood - lastNumCollectedFood);
        lastNumCollectedFood = currNumCollectedFood;
        last_time_in_minutes++;
    }
    UpdatePheromoneList();

	if(GetSpace().GetSimulationClock() > ResourceDensityDelay) {
      for(size_t i = 0; i < FoodColoringList.size(); i++) {
            FoodColoringList[i] = argos::CColor::BLACK;
      }
	}
	argos::CVector2 position;
    argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
    
    robotPosList.clear();
    for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
      argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
      BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
      CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
      position = c2.GetPosition();
      robotPosList[c2.GetId()] = position;
      //robotPosList.push_back(position);
    }
    
    //for(map<string, CVector2>::iterator it= robotPosList.begin(); it!=robotPosList.end(); ++it) {
	//	argos::LOG << "pos["<< it->first <<"]="<< it->second << endl;
	//}
         
    if(FoodList.size() == 0) {
	FidelityList.clear();
	PheromoneList.clear();
        TargetRayList.clear();
        Trajectory.clear();
    }
    
    // Export grid to CSV every 10 seconds for visualization
    static argos::Real lastExportTime = 0.0;
    static bool directoryCreated = false;
    argos::Real currentTime = getSimTimeInSeconds();
    
    if(currentTime - lastExportTime >= 10.0) {
        // Create heatmap_data directory on first export
        if(!directoryCreated) {
            createDirectoryIfNotExists("heatmap_data");
            directoryCreated = true;
        }
        
        std::string filename = "heatmap_data/grid_heatmap_" + std::to_string((int)currentTime) + ".csv";
        exportGridToCSV(filename);
        lastExportTime = currentTime;
    }
    
    // Export visited positions to CSV every 10 seconds for dot plot visualization
    static argos::Real lastDotplotExportTime = 0.0;
    static bool dotplotDirectoryCreated = false;
    
    if(currentTime - lastDotplotExportTime >= 10.0) {
        // Create dotplot_data directory on first export
        if(!dotplotDirectoryCreated) {
            createDirectoryIfNotExists("dotplot_data");
            dotplotDirectoryCreated = true;
        }
        
        std::string dotplot_filename = "dotplot_data/visited_positions_" + std::to_string((int)currentTime) + ".csv";
        exportVisitedPositionsToCSV(dotplot_filename);
        lastDotplotExportTime = currentTime;
    }
    
    // Export food locations to CSV every 10 seconds for food visualization
    static argos::Real lastFoodExportTime = 0.0;
    static bool foodDirectoryCreated = false;
    
    if(currentTime - lastFoodExportTime >= 10.0) {
        // Create food_data directory on first export
        if(!foodDirectoryCreated) {
            createDirectoryIfNotExists("food_data");
            foodDirectoryCreated = true;
        }
        
        std::string food_filename = "food_data/food_locations_" + std::to_string((int)currentTime) + ".csv";
        exportFoodLocationsToCSV(food_filename);
        lastFoodExportTime = currentTime;
    }
}

void CPFA_loop_functions::PostStep() {
	// PostStep logic can be added here if needed
}

bool CPFA_loop_functions::IsExperimentFinished() {
	bool isFinished = false;

	// Check if 88% of food has been collected
	// if(score >= FoodTarget88Percent) {
	// 	isFinished = true;
	// 	argos::LOG << "Simulation finished: Collected " << score << "/" << FoodList.size() + score << " food items (target: " << FoodTarget88Percent << ", 88%)" << std::endl;
	// }
	// Fallback: if all food is collected (100%)
	if(FoodList.size() == 0) {
		isFinished = true;
		
		// Record 100% milestone if we haven't already
		if (lastMilestone < 10) {
			argos::Real currentTime = getSimTimeInSeconds();
			// Add any missing milestones up to 100%
			for (size_t m = lastMilestone + 1; m <= 10; m++) {
				resourceCollectionMilestones.push_back(currentTime);
				argos::LOG << "Milestone reached: " << (m * 10) << "% of resources collected at time " 
						   << currentTime << " seconds (final collection)" << std::endl;
			}
			lastMilestone = 10;
		}
		
		argos::LOG << "Simulation finished: All food collected (100%)" << std::endl;
	}
	// else if(GetSpace().GetSimulationClock() >= MaxSimTime) {
	// 	isFinished = true;
	// }
         
         
    
	if(isFinished == true && MaxSimCounter > 1) {
		size_t newSimCounter = SimCounter + 1;
		size_t newMaxSimCounter = MaxSimCounter - 1;
        argos::LOG<< "time out..."<<endl; 
		PostExperiment();
		Reset();

		SimCounter    = newSimCounter;
		MaxSimCounter = newMaxSimCounter;
		isFinished    = false;
	}

	return isFinished;
}

void CPFA_loop_functions::PostExperiment() {
	  
     // Calculate cells visited metric
     size_t cellsVisited = 0;
     for(size_t i = 0; i < GridHeight; i++) {
         for(size_t j = 0; j < GridWidth; j++) {
             if(Grid[i][j] > 0) {
                 cellsVisited++;
             }
         }
     }
     size_t totalCells = GridWidth * GridHeight;
	  
     printf("%f, %f, %lu\n", score, getSimTimeInSeconds(), RandomSeed);
     printf("Total cells visited: %lu / %lu\n", cellsVisited, totalCells);
	 double avg = totalVisitedPositionsCount / (double)timesreceivedRobotMemories;
	 printf("Average memories recieved per return: %f\n", avg); //average number of positions received per robot
	 printf("Total unique positions visited: %lu\n", totalVisitedPositionsCount);
     
     // Export resource collection milestones to CSV
     if (!resourceCollectionMilestones.empty()) {
         // Create milestone_data directory if it doesn't exist
         createDirectoryIfNotExists("milestone_data");
         std::string milestoneFilename = "milestone_data/resource_milestones_" + std::to_string(RandomSeed) + ".csv";
         exportResourceMilestonesToCSV(milestoneFilename);
     }
       
                  
    // if (PrintFinalScore == 1) {
    //     string type="";
    //     if (FoodDistribution == 0) type = "random";
    //     else if (FoodDistribution == 1) type = "cluster";
    //     else type = "powerlaw";
            
    //     ostringstream num_tag;
    //     num_tag << FoodItemCount; 
              
    //     ostringstream num_robots;
    //     num_robots <<  Num_robots;
   
    //     ostringstream arena_width;
    //     arena_width << ArenaWidth;
        
    //     ostringstream quardArena;
    //     if(abs(NestPosition.GetX())>=1){ //the central nest is not in the center, this is a quard arena
    //          quardArena << 1;
    //      }
    //      else{
    //          quardArena << 0;
    //     }
        
    //     string header = "./results/"+ type+"_CPFA_r"+num_robots.str()+"_tag"+num_tag.str()+"_"+arena_width.str()+"by"+arena_width.str()+"_quard_arena_" + quardArena.str() +"_";
       
    //     unsigned int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();//qilu 02/06/2021
       
    //     /* Real total_travel_time=0;
    //     Real total_search_time=0;
    //     ofstream travelSearchTimeDataOutput((header+"TravelSearchTimeData.txt").c_str(), ios::app);
    //     */
        
        
    //     argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
         
    //     for(argos::CSpace::TMapPerType::iterator it = footbots.begin(); it != footbots.end(); it++) {
    //         argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(it->second);
    //         BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
    //         CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
    //         CollisionTime += c2.GetCollisionTime();
            
    //         /*if(c2.GetStatus() == "SEARCHING"){
    //             total_search_time += SimTime-c2.GetTravelingTime();
    //             total_travel_time += c2.GetTravelingTime();
	//     }
    //         else {
	// 	total_search_time += c2.GetSearchingTime();
	// 	total_travel_time += SimTime-c2.GetSearchingTime();
    //         } */        
    //     }
    //     //travelSearchTimeDataOutput<< total_travel_time/ticks_per_second<<", "<<total_search_time/ticks_per_second<<endl;
    //     //travelSearchTimeDataOutput.close();   
             
    //     ofstream dataOutput( (header+ "iAntTagData.txt").c_str(), ios::app);
    //     // output to file
    //     if(dataOutput.tellp() == 0) {
    //         dataOutput << "tags_collected, collisions_in_seconds, time_in_minutes, random_seed\n";//qilu 08/18
    //     }
    
    //     //dataOutput <<data.CollisionTime/16.0<<", "<< time_in_minutes << ", " << data.RandomSeed << endl;
    //     //dataOutput << Score() << ", "<<(CollisionTime-16*Score())/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
    //     dataOutput << Score() << ", "<<CollisionTime/(2*ticks_per_second)<< ", "<< curr_time_in_minutes <<", "<<RandomSeed<<endl;
    //     dataOutput.close();
    
    //     ofstream forageDataOutput((header+"ForageData.txt").c_str(), ios::app);
    //     if(ForageList.size()!=0) forageDataOutput<<"Forage: "<< ForageList[0];
    //     for(size_t i=1; i< ForageList.size(); i++) forageDataOutput<<", "<<ForageList[i];
    //     forageDataOutput<<"\n";
    //     forageDataOutput.close();
        
    //     ofstream trajOutput( (header+ "iAntTrajData.txt").c_str(), ios::app);
    //     // output to file
    //     //if(trajOutput.tellp() == 0) {
    //         trajOutput << "trajs\n";//qilu 11/2023
    //     //}
        
    //     for(map<string, std::vector<CVector2>>::iterator it= Trajectory.begin(); it!= Trajectory.end(); ++it) {
			
	// 		for(size_t j = 0; j < it->second.size(); j++) {
	// 			trajOutput << it->second[j]<<"; ";
	// 		}
	// 		trajOutput << "\n";
		
	// 	}
        
	// 	trajOutput.close();
        
    //   }  

}


argos::CColor CPFA_loop_functions::GetFloorColor(const argos::CVector2 &c_pos_on_floor) {
	return argos::CColor::WHITE;
}

void CPFA_loop_functions::UpdatePheromoneList() {
	// Return if this is not a tick that lands on a 0.5 second interval
	if ((int)(GetSpace().GetSimulationClock()) % ((int)(GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick()) / 2) != 0) return;
	
	std::vector<Pheromone> new_p_list; 

	argos::Real t = GetSpace().GetSimulationClock() / GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick();

	//ofstream log_output_stream;
	//log_output_stream.open("time.txt", ios::app);
	//log_output_stream << t << ", " << GetSpace().GetSimulationClock() << ", " << GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() << endl;
	//log_output_stream.close();
	    for(size_t i = 0; i < PheromoneList.size(); i++) {

		PheromoneList[i].Update(t);
		if(PheromoneList[i].IsActive()) {
			new_p_list.push_back(PheromoneList[i]);
		}
      }
     	PheromoneList = new_p_list;
	new_p_list.clear();
}
void CPFA_loop_functions::SetFoodDistribution() {
	switch(FoodDistribution) {
		case 0:
			RandomFoodDistribution();
			break;
		case 1:
			ClusterFoodDistribution();
			break;
		case 2:
			PowerLawFoodDistribution();
			break;
		default:
			argos::LOGERR << "ERROR: Invalid food distribution in XML file.\n";
	}
}

void CPFA_loop_functions::RandomFoodDistribution() {
	FoodList.clear();
        FoodColoringList.clear();
        ClusterCenters.clear(); // No clusters in random distribution
	argos::CVector2 placementPosition;

	for(size_t i = 0; i < FoodItemCount; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, 1, 1)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		FoodList.push_back(placementPosition);
		FoodColoringList.push_back(argos::CColor::BLACK);
	}
}

 
void CPFA_loop_functions::ClusterFoodDistribution() {
        FoodList.clear();
        ClusterCenters.clear();
	argos::Real     foodOffset  = 3.0 * FoodRadius;
	size_t          foodToPlace = NumberOfClusters * ClusterWidthX * ClusterWidthY;
	size_t          foodPlaced = 0;
	argos::CVector2 placementPosition;

	FoodItemCount = foodToPlace;

	for(size_t i = 0; i < NumberOfClusters; i++) {
		placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

		while(IsOutOfBounds(placementPosition, ClusterWidthY, ClusterWidthX)) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));
		}

		// Store the cluster center (bottom-left corner of the cluster)
		ClusterCenters.push_back(placementPosition);

		for(size_t j = 0; j < ClusterWidthY; j++) {
			for(size_t k = 0; k < ClusterWidthX; k++) {
				foodPlaced++;
				/*
				#include <argos3/plugins/simulator/entities/box_entity.h>

				string label("my_box_");
				label.push_back('0' + foodPlaced++);

				CBoxEntity *b = new CBoxEntity(label,
					CVector3(placementPosition.GetX(),
					placementPosition.GetY(), 0.0), CQuaternion(), true,
					CVector3(0.1, 0.1, 0.001), 1.0);
				AddEntity(*b);
				*/

				FoodList.push_back(placementPosition);
				FoodColoringList.push_back(argos::CColor::BLACK);
				placementPosition.SetX(placementPosition.GetX() + foodOffset);
			}

			placementPosition.SetX(placementPosition.GetX() - (ClusterWidthX * foodOffset));
			placementPosition.SetY(placementPosition.GetY() + foodOffset);
		}
	}
}


void CPFA_loop_functions::PowerLawFoodDistribution() {
 FoodList.clear();
    FoodColoringList.clear();
    ClusterCenters.clear(); // Clear clusters (PowerLaw has its own cluster structure that we'd need to handle separately)
	argos::Real foodOffset     = 3.0 * FoodRadius;
	size_t      foodPlaced     = 0;
	size_t      powerLawLength = 1;
	size_t      maxTrials      = 200;
	size_t      trialCount     = 0;

	std::vector<size_t> powerLawClusters;
	std::vector<size_t> clusterSides;
	argos::CVector2     placementPosition;

    //-----Wayne: Dertermine PowerRank and food per PowerRank group
    size_t priorPowerRank = 0;
    size_t power4 = 0;
    size_t FoodCount = 0;
    size_t diffFoodCount = 0;
    size_t singleClusterCount = 0;
    size_t otherClusterCount = 0;
    size_t modDiff = 0;
    
    //Wayne: priorPowerRank is determined by what power of 4
    //plus a multiple of power4 increases the food count passed required count
    //this is how powerlaw works to divide up food into groups
    //the number of groups is the powerrank
    while (FoodCount < FoodItemCount){
        priorPowerRank++;
        power4 = pow (4.0, priorPowerRank);
        FoodCount = power4 + priorPowerRank * power4;
    }
    
    //Wayne: Actual powerRank is prior + 1
    PowerRank = priorPowerRank + 1;
    
    //Wayne: Equalizes out the amount of food in each group, with the 1 cluster group taking the
    //largest loss if not equal, when the powerrank is not a perfect fit with the amount of food.
    diffFoodCount = FoodCount - FoodItemCount;
    modDiff = diffFoodCount % PowerRank;
    
    if (FoodItemCount % PowerRank == 0){
        singleClusterCount = FoodItemCount / PowerRank;
        otherClusterCount = singleClusterCount;
    }
    else {
        otherClusterCount = FoodItemCount / PowerRank + 1;
        singleClusterCount = otherClusterCount - modDiff;
    }
    //-----Wayne: End of PowerRank and food per PowerRank group
    
	for(size_t i = 0; i < PowerRank; i++) {
		powerLawClusters.push_back(powerLawLength * powerLawLength);
		powerLawLength *= 2;
	}

	for(size_t i = 0; i < PowerRank; i++) {
		powerLawLength /= 2;
		clusterSides.push_back(powerLawLength);
	}
    /*Wayne: Modified to break from loops if food count reached.
     Provides support for unequal clusters and odd food numbers.
     Necessary for DustUp and Jumble Distribution changes. */
    
	for(size_t h = 0; h < powerLawClusters.size(); h++) {
		for(size_t i = 0; i < powerLawClusters[h]; i++) {
			placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

			while(IsOutOfBounds(placementPosition, clusterSides[h], clusterSides[h])) {
				trialCount++;
				placementPosition.Set(RNG->Uniform(ForageRangeX), RNG->Uniform(ForageRangeY));

				if(trialCount > maxTrials) {
					argos::LOGERR << "PowerLawDistribution(): Max trials exceeded!\n";
					break;
				}
			}

            trialCount = 0;
			for(size_t j = 0; j < clusterSides[h]; j++) {
				for(size_t k = 0; k < clusterSides[h]; k++) {
					foodPlaced++;
					FoodList.push_back(placementPosition);
					FoodColoringList.push_back(argos::CColor::BLACK);
					placementPosition.SetX(placementPosition.GetX() + foodOffset);
                    if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
				}

				placementPosition.SetX(placementPosition.GetX() - (clusterSides[h] * foodOffset));
				placementPosition.SetY(placementPosition.GetY() + foodOffset);
                if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
            if (foodPlaced == singleClusterCount + h * otherClusterCount) break;
			}
		}
	FoodItemCount = foodPlaced;
}
 
bool CPFA_loop_functions::IsOutOfBounds(argos::CVector2 p, size_t length, size_t width) {
	argos::CVector2 placementPosition = p;

	argos::Real foodOffset   = 3.0 * FoodRadius;
	argos::Real widthOffset  = 3.0 * FoodRadius * (argos::Real)width;
	argos::Real lengthOffset = 3.0 * FoodRadius * (argos::Real)length;

	argos::Real x_min = p.GetX() - FoodRadius;
	argos::Real x_max = p.GetX() + FoodRadius + widthOffset;

	argos::Real y_min = p.GetY() - FoodRadius;
	argos::Real y_max = p.GetY() + FoodRadius + lengthOffset;

	if((x_min < (ForageRangeX.GetMin() + FoodRadius))
			|| (x_max > (ForageRangeX.GetMax() - FoodRadius)) ||
			(y_min < (ForageRangeY.GetMin() + FoodRadius)) ||
			(y_max > (ForageRangeY.GetMax() - FoodRadius)))
	{
		return true;
	}

	for(size_t j = 0; j < length; j++) {
		for(size_t k = 0; k < width; k++) {
			if(IsCollidingWithFood(placementPosition)) return true;
			if(IsCollidingWithNest(placementPosition)) return true;
			placementPosition.SetX(placementPosition.GetX() + foodOffset);
		}

		placementPosition.SetX(placementPosition.GetX() - (width * foodOffset));
		placementPosition.SetY(placementPosition.GetY() + foodOffset);
	}

	return false;
}

  
bool CPFA_loop_functions::IsCollidingWithNest(argos::CVector2 p) {
	argos::Real nestRadiusPlusBuffer = NestRadius + FoodRadius;
	argos::Real NRPB_squared = nestRadiusPlusBuffer * nestRadiusPlusBuffer;

      return ( (p - NestPosition).SquareLength() < NRPB_squared) ;
}

bool CPFA_loop_functions::IsCollidingWithFood(argos::CVector2 p) {
	argos::Real foodRadiusPlusBuffer = 2.0 * FoodRadius;
	argos::Real FRPB_squared = foodRadiusPlusBuffer * foodRadiusPlusBuffer;

	for(size_t i = 0; i < FoodList.size(); i++) {
		if((p - FoodList[i]).SquareLength() < FRPB_squared) return true;
	}

	return false;
}

unsigned int CPFA_loop_functions::getNumberOfRobots() {
	return GetSpace().GetEntitiesByType("foot-bot").size();
}

double CPFA_loop_functions::getProbabilityOfSwitchingToSearching() {
	return ProbabilityOfSwitchingToSearching;
}

double CPFA_loop_functions::getProbabilityOfReturningToNest() {
	return ProbabilityOfReturningToNest;
}

// Value in Radians
double CPFA_loop_functions::getUninformedSearchVariation() {
	return UninformedSearchVariation.GetValue();
}

double CPFA_loop_functions::getRateOfInformedSearchDecay() {
	return RateOfInformedSearchDecay;
}

double CPFA_loop_functions::getRateOfSiteFidelity() {
	return RateOfSiteFidelity;
}

double CPFA_loop_functions::getRateOfLayingPheromone() {
	return RateOfLayingPheromone;
}

double CPFA_loop_functions::getRateOfPheromoneDecay() {
	return RateOfPheromoneDecay;
}

int CPFA_loop_functions::getSearchAlgorithmMode() {
	return SearchAlgorithmMode;
}

void CPFA_loop_functions::incrementRejectedLocationCounter() {
	RejectedLocationCounter++;
}

argos::Real CPFA_loop_functions::getSimTimeInSeconds() {
	int ticks_per_second = GetSimulator().GetPhysicsEngine("dyn2d").GetInverseSimulationClockTick(); //qilu 02/06/2021
	float sim_time = GetSpace().GetSimulationClock();
	return sim_time/ticks_per_second;
}

void CPFA_loop_functions::SetTrial(unsigned int v) {
}

void CPFA_loop_functions::setScore(double s) {
	score = s;
    
    // Record milestone when score changes
    recordResourceMilestone(static_cast<size_t>(score));
    
	if (score >= NumDistributedFood) {
		PostExperiment();
	}
}

double CPFA_loop_functions::Score() {	
	return score;
}

void CPFA_loop_functions::increaseNumDistributedFoodByOne(){
    NumDistributedFood++;
}

void CPFA_loop_functions::ConfigureFromGenome(Real* g)
{
	// Assign genome generated by the GA to the appropriate internal variables.
	ProbabilityOfSwitchingToSearching = g[0];
	ProbabilityOfReturningToNest      = g[1];
	UninformedSearchVariation.SetValue(g[2]);
	RateOfInformedSearchDecay         = g[3];
	RateOfSiteFidelity                = g[4];
	RateOfLayingPheromone             = g[5];
	RateOfPheromoneDecay              = g[6];
}

void CPFA_loop_functions::create_grid(argos::Real cell_size) {
	// Store the cell size
	CellSize = cell_size;
	
	// Get arena dimensions
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real arena_width = ArenaSize.GetX();
	argos::Real arena_height = ArenaSize.GetY();
	
	// Calculate grid dimensions by dividing arena size by cell size
	GridWidth = static_cast<size_t>(std::ceil(arena_width / cell_size));
	GridHeight = static_cast<size_t>(std::ceil(arena_height / cell_size));
	
	// Initialize the 2D grid with zeros
	Grid.clear();
	Grid.resize(GridHeight, std::vector<int>(GridWidth, 0));
	
	// Print grid information
	argos::LOG << "Grid created with parameters:" << std::endl;
	argos::LOG << "  Arena size: " << arena_width << " x " << arena_height << std::endl;
	argos::LOG << "  Cell size: " << cell_size << std::endl;
	argos::LOG << "  Grid dimensions: " << GridWidth << " x " << GridHeight << " cells" << std::endl;
	argos::LOG << "  Total cells: " << GridWidth * GridHeight << std::endl;
	
	// Print the grid visualization (showing a sample if it's too large)
	// argos::LOG << "Grid visualization (all cells initialized to 0):" << std::endl;
	
	// If grid is small enough, print the entire grid
	// Print from top to bottom to match world coordinate system (higher Y values first)
	// if (GridWidth <= 20 && GridHeight <= 20) {
	// 	for (int i = GridHeight - 1; i >= 0; i--) {
	// 		std::string row = "";
	// 		for (size_t j = 0; j < GridWidth; j++) {
	// 			row += std::to_string(Grid[i][j]) + " ";
	// 		}
	// 		argos::LOG << row << std::endl;
	// 	}
	// } else {
	// 	// For larger grids, show top 10x10 section (from higher Y values down)
	// 	argos::LOG << "Grid is large (" << GridWidth << "x" << GridHeight << "), showing top 10x10 section:" << std::endl;
	// 	size_t max_rows = std::min(GridHeight, static_cast<size_t>(10));
	// 	size_t max_cols = std::min(GridWidth, static_cast<size_t>(10));
		
	// 	// Start from the top rows (higher Y values) and work down
	// 	for (int i = GridHeight - 1; i >= static_cast<int>(GridHeight - max_rows); i--) {
	// 		std::string row = "";
	// 		for (size_t j = 0; j < max_cols; j++) {
	// 			row += std::to_string(Grid[i][j]) + " ";
	// 		}
	// 		if (GridWidth > 10) row += "...";
	// 		argos::LOG << row << std::endl;
	// 	}
	// 	if (GridHeight > 10) {
	// 		argos::LOG << "..." << std::endl;
	// 	}
	// }
}

void CPFA_loop_functions::receiveRobotMemory(const std::string& robotId, const std::vector<argos::CVector2>& robotMemory) {
	// argos::LOG << "Receiving robot memory from " << robotId << " with " << robotMemory.size() << " locations" << std::endl;
	
	// Store the received memory so we could remember the exact positions the robots have visited
	for(const auto& pos : robotMemory) {
		VisitedPositions.push_back(pos);
	}
	totalVisitedPositionsCount += robotMemory.size();
	timesreceivedRobotMemories++;



	// Get arena dimensions to convert world coordinates to grid coordinates
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real arena_width = ArenaSize.GetX();
	argos::Real arena_height = ArenaSize.GetY();
	
	// Arena coordinates go from -arena_width/2 to +arena_width/2 and -arena_height/2 to +arena_height/2
	argos::Real half_width = arena_width / 2.0;
	argos::Real half_height = arena_height / 2.0;
	
	// Process each location in robot memory
	for(size_t i = 0; i < robotMemory.size(); i++) {
		argos::CVector2 location = robotMemory[i];
		
		// Convert world coordinates to grid coordinates for heatmap
		// World coordinates: (-half_width, -half_height) to (+half_width, +half_height)
		// Grid coordinates: (0, 0) to (GridWidth-1, GridHeight-1)
		
		// Translate from world coordinates to grid coordinates
		argos::Real normalized_x = (location.GetX() + half_width) / arena_width;  // 0 to 1
		argos::Real normalized_y = (location.GetY() + half_height) / arena_height; // 0 to 1
		
		// Convert to grid indices
		int grid_x = static_cast<int>(normalized_x * GridWidth);
		int grid_y = static_cast<int>(normalized_y * GridHeight);
		
		// Clamp to valid grid boundaries (safety check)
		grid_x = std::max(0, std::min(grid_x, static_cast<int>(GridWidth - 1)));
		grid_y = std::max(0, std::min(grid_y, static_cast<int>(GridHeight - 1)));
		
		// Increment the grid cell count
		Grid[grid_y][grid_x]++;
		
		// argos::LOG << "  Location " << (i+1) << ": " << location 
		// 		   << " -> Grid[" << grid_y << "][" << grid_x << "] = " << Grid[grid_y][grid_x] << std::endl;
	}
	
	// Optional: Print updated grid section if it's small enough
	// Print from top to bottom to match world coordinate system (higher Y values first)
	// if (GridWidth <= 10 && GridHeight <= 10) {
	// 	argos::LOG << "Updated grid after processing " << robotId << " memory:" << std::endl;
	// 	for (int i = GridHeight - 1; i >= 0; i--) {
	// 		std::string row = "";
	// 		for (size_t j = 0; j < GridWidth; j++) {
	// 			row += std::to_string(Grid[i][j]) + " ";
	// 		}
	// 		argos::LOG << row << std::endl;
	// 	}
	// }
}

int CPFA_loop_functions::getGridVisitCount(argos::CVector2 worldPosition) {
	// Get arena dimensions to convert world coordinates to grid coordinates
	argos::CVector3 ArenaSize = GetSpace().GetArenaSize();
	argos::Real arena_width = ArenaSize.GetX();
	argos::Real arena_height = ArenaSize.GetY();
	
	// Arena coordinates go from -arena_width/2 to +arena_width/2 and -arena_height/2 to +arena_height/2
	argos::Real half_width = arena_width / 2.0;
	argos::Real half_height = arena_height / 2.0;
	
	// Convert world coordinates to grid coordinates
	// World coordinates: (-half_width, -half_height) to (+half_width, +half_height)
	// Grid coordinates: (0, 0) to (GridWidth-1, GridHeight-1)
	
	// Translate from world coordinates to grid coordinates
	argos::Real normalized_x = (worldPosition.GetX() + half_width) / arena_width;  // 0 to 1
	argos::Real normalized_y = (worldPosition.GetY() + half_height) / arena_height; // 0 to 1
	
	// Convert to grid indices
	int grid_x = static_cast<int>(normalized_x * GridWidth);
	int grid_y = static_cast<int>(normalized_y * GridHeight);
	
	// Clamp to valid grid boundaries (safety check)
	grid_x = std::max(0, std::min(grid_x, static_cast<int>(GridWidth - 1)));
	grid_y = std::max(0, std::min(grid_y, static_cast<int>(GridHeight - 1)));
	
	// Return the visit count for this grid cell
	return Grid[grid_y][grid_x];
}

void CPFA_loop_functions::exportGridToCSV(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		argos::LOGERR << "Failed to open file for grid export: " << filename << std::endl;
		return;
	}
	
	// Write header with metadata
	file << "# Grid Export - Simulation Time: " << getSimTimeInSeconds() << " seconds" << std::endl;
	file << "# Grid Dimensions: " << GridWidth << "x" << GridHeight << std::endl;
	file << "# Cell Size: " << CellSize << " meters" << std::endl;
	
	// Write column headers (grid x coordinates)
	file << "y\\x";
	for (size_t j = 0; j < GridWidth; j++) {
		file << "," << j;
	}
	file << std::endl;
	
	// Write grid data with row headers (grid y coordinates)
	for (size_t i = 0; i < GridHeight; i++) {
		file << i; // Row header
		for (size_t j = 0; j < GridWidth; j++) {
			file << "," << Grid[i][j];
		}
		file << std::endl;
	}
	
	file.close();
	// argos::LOG << "Grid exported to: " << filename << std::endl;
}

void CPFA_loop_functions::exportVisitedPositionsToCSV(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		argos::LOGERR << "Failed to open file for visited positions export: " << filename << std::endl;
		return;
	}
	
	// Write header with metadata
	file << "# Visited Positions Export - Simulation Time: " << getSimTimeInSeconds() << " seconds" << std::endl;
	file << "# Total Positions: " << VisitedPositions.size() << std::endl;
	file << "# Food Distribution: " << FoodDistribution << std::endl;
	file << "# Number of Clusters: " << ClusterCenters.size() << std::endl;
	file << "# Cluster Dimensions: " << ClusterWidthX << "x" << ClusterWidthY << std::endl;
	file << "# Food Radius: " << FoodRadius << std::endl;
	file << "# Format: X,Y (coordinates in meters)" << std::endl;
	
	// Write cluster information if available
	if (!ClusterCenters.empty()) {
		file << "# Cluster Centers (bottom-left corners):" << std::endl;
		for(size_t i = 0; i < ClusterCenters.size(); i++) {
			file << "# Cluster " << i << ": " << ClusterCenters[i].GetX() << "," << ClusterCenters[i].GetY() << std::endl;
		}
	}
	
	file << "# === VISITED POSITIONS DATA ===" << std::endl;
	file << "X,Y" << std::endl;
	
	// Write all visited positions
	for(const auto& pos : VisitedPositions) {
		file << pos.GetX() << "," << pos.GetY() << std::endl;
	}
	
	file.close();
	// argos::LOG << "Visited positions exported to: " << filename << " (" << VisitedPositions.size() << " positions, " << ClusterCenters.size() << " clusters)" << std::endl;
}

bool CPFA_loop_functions::createDirectoryIfNotExists(const std::string& dirPath) {
	struct stat info;
	
	// Check if directory already exists
	if (stat(dirPath.c_str(), &info) == 0) {
		if (info.st_mode & S_IFDIR) {
			return true; // Directory exists
		}
	}
	
	// Try to create directory
	if (mkdir(dirPath.c_str(), 0755) == 0) {
		argos::LOG << "Created directory: " << dirPath << std::endl;
		return true;
	} else {
		argos::LOGERR << "Failed to create directory: " << dirPath << std::endl;
		return false;
	}
}

void CPFA_loop_functions::clearHeatmapData() {
	const std::string heatmapDir = "heatmap_data";
	
	// Check if directory exists
	struct stat info;
	if (stat(heatmapDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
		// Directory doesn't exist, nothing to clear
		return;
	}
	
	// Open directory
	DIR* dir = opendir(heatmapDir.c_str());
	if (dir == nullptr) {
		argos::LOGERR << "Failed to open heatmap_data directory for cleaning" << std::endl;
		return;
	}
	
	// Read directory entries and delete CSV files
	struct dirent* entry;
	int filesDeleted = 0;
	
	while ((entry = readdir(dir)) != nullptr) {
		// Skip . and .. entries
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}
		
		// Check if it's a CSV file
		std::string filename = entry->d_name;
		if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".csv") {
			std::string fullPath = heatmapDir + "/" + filename;
			if (remove(fullPath.c_str()) == 0) {
				filesDeleted++;
			} else {
				argos::LOGERR << "Failed to delete: " << fullPath << std::endl;
			}
		}
	}
	
	closedir(dir);
	
	if (filesDeleted > 0) {
		argos::LOG << "Cleared heatmap data: deleted " << filesDeleted << " CSV files" << std::endl;
	} else {
		argos::LOG << "Heatmap data directory is already clean" << std::endl;
	}
}

void CPFA_loop_functions::clearDotplotData() {
	const std::string dotplotDir = "dotplot_data";
	
	// Check if directory exists
	struct stat info;
	if (stat(dotplotDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
		// Directory doesn't exist, nothing to clear
		return;
	}
	
	// Open directory
	DIR* dir = opendir(dotplotDir.c_str());
	if (dir == nullptr) {
		argos::LOGERR << "Failed to open dotplot_data directory for cleaning" << std::endl;
		return;
	}
	
	// Read directory entries and delete CSV files
	struct dirent* entry;
	int filesDeleted = 0;
	
	while ((entry = readdir(dir)) != nullptr) {
		// Skip . and .. entries
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}
		
		// Check if it's a CSV file
		std::string filename = entry->d_name;
		if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".csv") {
			std::string fullPath = dotplotDir + "/" + filename;
			if (remove(fullPath.c_str()) == 0) {
				filesDeleted++;
			} else {
				argos::LOGERR << "Failed to delete: " << fullPath << std::endl;
			}
		}
	}
	
	closedir(dir);
	
	if (filesDeleted > 0) {
		argos::LOG << "Cleared dotplot data: deleted " << filesDeleted << " CSV files" << std::endl;
	} else {
		argos::LOG << "Dotplot data directory is already clean" << std::endl;
	}
}

void CPFA_loop_functions::clearTrajectoryData() {
    const std::string trajectoryDir = "trajectory_data";
    
    // Check if directory exists
    struct stat info;
    if (stat(trajectoryDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
        // Directory doesn't exist, nothing to clear
        return;
    }
    
    // Open directory
    DIR* dir = opendir(trajectoryDir.c_str());
    if (dir == nullptr) {
        argos::LOGERR << "Failed to open trajectory_data directory for cleaning" << std::endl;
        return;
    }
    
    // Read directory entries and delete CSV files
    struct dirent* entry;
    int filesDeleted = 0;
    
    while ((entry = readdir(dir)) != nullptr) {
        // Skip . and .. entries
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }
        
        // Check if it's a CSV file
        std::string filename = entry->d_name;
        if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".csv") {
            std::string fullPath = trajectoryDir + "/" + filename;
            if (remove(fullPath.c_str()) == 0) {
                filesDeleted++;
            } else {
                argos::LOGERR << "Failed to delete: " << fullPath << std::endl;
            }
        }
    }
    
    closedir(dir);
    
    if (filesDeleted > 0) {
        argos::LOG << "Cleared trajectory data: deleted " << filesDeleted << " CSV files" << std::endl;
    } else {
        argos::LOG << "Trajectory data directory is already clean" << std::endl;
    }
}

void CPFA_loop_functions::exportRandomSearchTrajectory(const std::string& robotId, const std::vector<argos::CVector2>& trajectory, const std::vector<argos::CVector2>& centerPoints, const argos::CVector2& targetPosition) {
	if (trajectory.empty()) return;
	
	// Create trajectory_data directory if it doesn't exist
	std::string dirPath = "trajectory_data";
	createDirectoryIfNotExists(dirPath);
	
	// Create filename with robot ID and timestamp
	argos::Real currentTime = getSimTimeInSeconds();
	std::ostringstream filename;
	filename << dirPath << "/" << robotId << "_trajectory_" << std::fixed << std::setprecision(1) << currentTime << ".csv";
	
	std::ofstream file(filename.str());
	if (file.is_open()) {
		// Write header with target position and point type
		// point_type: 0 = trajectory point, 1 = center point
		file << "x,y,target_x,target_y,point_type\n";
		
		// Write trajectory points
		for (const auto& pos : trajectory) {
			file << pos.GetX() << "," << pos.GetY() << "," << targetPosition.GetX() << "," << targetPosition.GetY() << ",0\n";
		}
		
		// Write center points
		for (const auto& pos : centerPoints) {
			file << pos.GetX() << "," << pos.GetY() << "," << targetPosition.GetX() << "," << targetPosition.GetY() << ",1\n";
		}
		
		file.close();
		// argos::LOG << "Exported trajectory for robot " << robotId << " with " << trajectory.size() << " points to " << filename.str() << std::endl;
	} else {
		argos::LOGERR << "Failed to open trajectory file: " << filename.str() << std::endl;
	}
}

void CPFA_loop_functions::exportFoodLocationsToCSV(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		argos::LOGERR << "Failed to open file for food locations export: " << filename << std::endl;
		return;
	}
	
	// Get food distance tolerance from one of the robot controllers
	argos::Real foodDistanceTolerance = 0.13; // Default value
	argos::CSpace::TMapPerType& footbots = GetSpace().GetEntitiesByType("foot-bot");
	if (!footbots.empty()) {
		argos::CFootBotEntity& footBot = *argos::any_cast<argos::CFootBotEntity*>(footbots.begin()->second);
		BaseController& c = dynamic_cast<BaseController&>(footBot.GetControllableEntity().GetController());
		CPFA_controller& c2 = dynamic_cast<CPFA_controller&>(c);
		foodDistanceTolerance = sqrt(c2.FoodDistanceTolerance); // It's stored as squared value
	}
	
	// Write header with metadata
	file << "# Food Locations Export - Simulation Time: " << getSimTimeInSeconds() << " seconds" << std::endl;
	file << "# Total Food Items: " << FoodList.size() << std::endl;
	file << "# Food Distribution: " << FoodDistribution << std::endl;
	file << "# Food Radius: " << FoodRadius << std::endl;
	file << "# Food Distance Tolerance: " << foodDistanceTolerance << std::endl;
	file << "# Random Seed: " << RandomSeed << std::endl;
	
	// Write cluster information if available (for clustered distribution)
	if (FoodDistribution == 1 && !ClusterCenters.empty()) {
		file << "# Number of Clusters: " << ClusterCenters.size() << std::endl;
		file << "# Cluster Dimensions: " << ClusterWidthX << "x" << ClusterWidthY << std::endl;
		for(size_t i = 0; i < ClusterCenters.size(); i++) {
			file << "# Cluster " << i << " Center: " << ClusterCenters[i].GetX() << "," << ClusterCenters[i].GetY() << std::endl;
		}
	}
	
	file << "# === FOOD LOCATIONS DATA ===" << std::endl;
	file << "X,Y,Status" << std::endl;
	
	// Write all food locations with their status
	for(size_t i = 0; i < FoodList.size(); i++) {
		// Check if this food has been collected by looking in CollectedFoodList
		bool isCollected = false;
		for(const auto& collectedFood : CollectedFoodList) {
			if((FoodList[i] - collectedFood).SquareLength() < 0.001) { // Very small tolerance for comparison
				isCollected = true;
				break;
			}
		}
		
		file << FoodList[i].GetX() << "," << FoodList[i].GetY() << "," 
			 << (isCollected ? "collected" : "available") << std::endl;
	}
	
	file.close();
	// argos::LOG << "Food locations exported to: " << filename << " (" << FoodList.size() << " items)" << std::endl;
}

void CPFA_loop_functions::clearFoodData() {
	const std::string foodDir = "food_data";
	
	// Check if directory exists
	struct stat info;
	if (stat(foodDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
		// Directory doesn't exist, nothing to clear
		return;
	}
	
	// Open directory
	DIR* dir = opendir(foodDir.c_str());
	if (dir == nullptr) {
		argos::LOGERR << "Failed to open food_data directory for cleaning" << std::endl;
		return;
	}
	
	// Read directory entries and delete CSV files
	struct dirent* entry;
	int filesDeleted = 0;
	
	while ((entry = readdir(dir)) != nullptr) {
		// Skip . and .. entries
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}
		
		// Check if it's a CSV file
		std::string filename = entry->d_name;
		if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".csv") {
			std::string fullPath = foodDir + "/" + filename;
			if (remove(fullPath.c_str()) == 0) {
				filesDeleted++;
			} else {
				argos::LOGERR << "Failed to delete: " << fullPath << std::endl;
			}
		}
	}
	
	closedir(dir);
	
	if (filesDeleted > 0) {
		argos::LOG << "Cleared food data: deleted " << filesDeleted << " CSV files" << std::endl;
	} else {
		argos::LOG << "Food data directory is already clean" << std::endl;
	}
}

void CPFA_loop_functions::recordResourceMilestone(size_t currentScore) {
	if (FoodList.size() == 0) return; // No food to track
	
	size_t totalFood = FoodList.size() + currentScore; // Total original food count
	if (totalFood == 0) return;
	
	// Calculate which milestone percentage this score represents
	double percentage = static_cast<double>(currentScore) / static_cast<double>(totalFood);
	size_t milestone = static_cast<size_t>(percentage * 10.0); // Convert to 0-10 range
	
	// Check if we've reached a new milestone (10%, 20%, 30%, etc.)
	if (milestone > lastMilestone && milestone <= 10) {
		// Record the time for all milestones between lastMilestone and current milestone
		for (size_t m = lastMilestone + 1; m <= milestone; m++) {
			if (m <= 10) { // Don't go beyond 100%
				argos::Real currentTime = getSimTimeInSeconds();
				resourceCollectionMilestones.push_back(currentTime);
				
				argos::LOG << "Milestone reached: " << (m * 10) << "% of resources collected at time " 
						   << currentTime << " seconds (score: " << currentScore << "/" << totalFood << ")" << std::endl;
			}
		}
		lastMilestone = milestone;
	}
}

void CPFA_loop_functions::exportResourceMilestonesToCSV(const std::string& filename) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		argos::LOGERR << "Failed to open file for resource milestones export: " << filename << std::endl;
		return;
	}
	
	// Write header with metadata
	file << "# Resource Collection Milestones - Random Seed: " << RandomSeed << std::endl;
	file << "# Food Distribution: " << FoodDistribution << std::endl;
	file << "# Total Food Items: " << (FoodList.size() + score) << std::endl;
	file << "# Algorithm Mode: " << SearchAlgorithmMode << std::endl;
	file << "# Number of Robots: " << Num_robots << std::endl;
	file << "# === MILESTONE DATA ===" << std::endl;
	file << "milestone_percent,time_seconds" << std::endl;
	
	// Write milestone data
	for (size_t i = 0; i < resourceCollectionMilestones.size(); i++) {
		size_t milestonePercent = (i + 1) * 10; // 10%, 20%, 30%, etc.
		file << milestonePercent << "," << resourceCollectionMilestones[i] << std::endl;
	}
	
	file.close();
	argos::LOG << "Resource milestones exported to: " << filename << " (" << resourceCollectionMilestones.size() << " milestones)" << std::endl;
}

void CPFA_loop_functions::clearMilestoneData() {
	const std::string milestoneDir = "milestone_data";
	
	// Check if directory exists
	struct stat info;
	if (stat(milestoneDir.c_str(), &info) != 0 || !(info.st_mode & S_IFDIR)) {
		// Directory doesn't exist, nothing to clear
		return;
	}
	
	// Open directory
	DIR* dir = opendir(milestoneDir.c_str());
	if (dir == nullptr) {
		argos::LOGERR << "Failed to open milestone_data directory for cleaning" << std::endl;
		return;
	}
	
	// Read directory entries and delete CSV files
	struct dirent* entry;
	int filesDeleted = 0;
	
	while ((entry = readdir(dir)) != nullptr) {
		// Skip . and .. entries
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}
		
		// Check if it's a CSV file
		std::string filename = entry->d_name;
		if (filename.size() > 4 && filename.substr(filename.size() - 4) == ".csv") {
			std::string fullPath = milestoneDir + "/" + filename;
			if (remove(fullPath.c_str()) == 0) {
				filesDeleted++;
			} else {
				argos::LOGERR << "Failed to delete: " << fullPath << std::endl;
			}
		}
	}
	
	closedir(dir);
	
	if (filesDeleted > 0) {
		argos::LOG << "Cleared milestone data: deleted " << filesDeleted << " CSV files" << std::endl;
	} else {
		argos::LOG << "Milestone data directory is already clean" << std::endl;
	}
}

void CPFA_loop_functions::SetSpiralOverlayPoints(const std::string& robotId, const std::vector<argos::CVector2>& points) {
	SpiralOverlayPoints[robotId] = points;
}

void CPFA_loop_functions::ClearSpiralOverlayPoints(const std::string& robotId) {
    SpiralOverlayPoints.erase(robotId);
}

void CPFA_loop_functions::AddSearchTrajectoryPoint(const std::string& robotId, const argos::CVector2& point) {
    SearchTrajectories[robotId].push_back(point);
}

void CPFA_loop_functions::ClearSearchTrajectory(const std::string& robotId) {
    SearchTrajectories.erase(robotId);
}REGISTER_LOOP_FUNCTIONS(CPFA_loop_functions, "CPFA_loop_functions")
