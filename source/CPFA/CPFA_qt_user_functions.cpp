#include "CPFA_qt_user_functions.h"

/*****
 * Constructor: In order for drawing functions in this class to be used by
 * ARGoS it must be registered using the RegisterUserFunction function.
 *****/
CPFA_qt_user_functions::CPFA_qt_user_functions() :
	loopFunctions(dynamic_cast<CPFA_loop_functions&>(CSimulator::GetInstance().GetLoopFunctions()))
{
	RegisterUserFunction<CPFA_qt_user_functions, CFootBotEntity>(&CPFA_qt_user_functions::DrawOnRobot);
	RegisterUserFunction<CPFA_qt_user_functions, CFloorEntity>(&CPFA_qt_user_functions::DrawOnArena);
}

void CPFA_qt_user_functions::DrawOnRobot(CFootBotEntity& entity) {
	CPFA_controller& c = dynamic_cast<CPFA_controller&>(entity.GetControllableEntity().GetController());
	//draw the simulated camera view
	/*std::vector<argos::CVector2> points;
	points.push_back(CVector2(0,0));
	points.push_back(CVector2(0.71,0.71));
	
	points.push_back(CVector2(0.83,0.55));
	
	points.push_back(CVector2(0.92, 0.38));
	points.push_back(CVector2(0.98,0.2));
	points.push_back(CVector2(1,0));
	
	points.push_back(CVector2(0.98,-0.2));
	
	points.push_back(CVector2(0.92, -0.38));
	
	points.push_back(CVector2(0.83, -0.55));
	points.push_back(CVector2(0.71, -0.71));
	 */

/* 
 points.push_back(CVector2(1.065,1.065));
	
	points.push_back(CVector2(1.245,0.825));
	
	points.push_back(CVector2(1.38, 0.57));
	points.push_back(CVector2(1.47,0.3));
	points.push_back(CVector2(1.5,0));
	
	points.push_back(CVector2(1.47,-0.3));
	
	points.push_back(CVector2(1.38, -0.57));
	
	points.push_back(CVector2(1.245, -0.825));
	points.push_back(CVector2(1.065, -1.065));

 */
	//DrawPolygon(CVector3(0, 0, 0.002), CQuaternion(), points, argos::CColor::RED, false);
	//points.clear();
	
	if(c.IsHoldingFood()) {
		DrawCylinder(CVector3(0.0, 0.0, 0.3), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	}

	if(loopFunctions.DrawIDs == 1) {
		/* Disable lighting, so it does not interfere with the chosen text color */
		glDisable(GL_LIGHTING);
		/* Disable face culling to be sure the text is visible from anywhere */
		glDisable(GL_CULL_FACE);
		/* Set the text color */
		CColor cColor(CColor::BLACK);
		glColor3ub(cColor.GetRed(), cColor.GetGreen(), cColor.GetBlue());

		/* The position of the text is expressed wrt the reference point of the footbot
		 * For a foot-bot, the reference point is the center of its base.
		 * See also the description in
		 * $ argos3 -q foot-bot
		 */
		
		// Disable for now
		//GetOpenGLWidget().renderText(0.0, 0.0, 0.5,             // position
		//			     entity.GetId().c_str()); // text
		
			DrawText(CVector3(0.0, 0.0, 0.3),   // position
            entity.GetId().c_str()); // text
		/* Restore face culling */
		glEnable(GL_CULL_FACE);
		/* Restore lighting */
		glEnable(GL_LIGHTING);
	}
}
 
void CPFA_qt_user_functions::DrawOnArena(CFloorEntity& entity) {
	DrawFood();
	DrawFidelity();
	DrawPheromones();
	DrawNest();
	DrawGrid();

	// Draw search trajectories for all robots in SEARCHING state
	for(const auto& robotTrajectory : loopFunctions.SearchTrajectories) {
		const std::vector<CVector2>& trajectory = robotTrajectory.second;
		// Draw trajectory lines
		for(size_t i = 1; i < trajectory.size(); i++) {
			const CVector2& pPrev = trajectory[i-1];
			const CVector2& pCurr = trajectory[i];
			CRay3 ray(CVector3(pPrev.GetX(), pPrev.GetY(), 0.005), CVector3(pCurr.GetX(), pCurr.GetY(), 0.005));
			DrawRay(ray, CColor::BLUE, 2.0);
		}
	}
	
	// Draw current spiral overlay points (current 9-group per robot)
	for(const auto& robotOverlay : loopFunctions.SpiralOverlayPoints) {
		const std::vector<CVector2>& points = robotOverlay.second;
		// Draw points
		for(size_t i = 0; i < points.size(); i++) {
			const CVector2& p2 = points[i];
			const CColor color = (i == 0 ? CColor::BLACK : CColor::MAGENTA);
			const Real radius = (i == 0 ? 0.06 : 0.04);
			DrawCylinder(CVector3(p2.GetX(), p2.GetY(), 0.004), CQuaternion(), radius, 0.01, color);
		}
		// Draw connecting lines between consecutive points
		for(size_t i = 1; i < points.size(); i++) {
			const CVector2& pPrev = points[i-1];
			const CVector2& pCurr = points[i];
			CRay3 ray(CVector3(pPrev.GetX(), pPrev.GetY(), 0.01), CVector3(pCurr.GetX(), pCurr.GetY(), 0.01));
			DrawRay(ray, CColor::MAGENTA, 1.0);
		}
	}

	if(loopFunctions.DrawTargetRays == 1) DrawTargetRays();
}

/*****
 * This function is called by the DrawOnArena(...) function. If the iAnt_data
 * object is not initialized this function should not be called.
 *****/
void CPFA_qt_user_functions::DrawNest() {
	/* 2d cartesian coordinates of the nest */
	Real x_coordinate = loopFunctions.NestPosition.GetX();
	Real y_coordinate = loopFunctions.NestPosition.GetY();

	/* required: leaving this 0.0 will draw the nest inside of the floor */
	Real elevation = loopFunctions.NestElevation;

	/* 3d cartesian coordinates of the nest */
	CVector3 nest_3d(x_coordinate, y_coordinate, elevation);

	/* Draw the nest on the arena. */
	//DrawCircle(nest_3d, CQuaternion(), loopFunctions.NestRadius, CColor::RED);
    DrawCylinder(nest_3d, CQuaternion(), loopFunctions.NestRadius, 0.008, CColor::GREEN);
}


void CPFA_qt_user_functions::DrawFood() {

	Real x, y;

	for(size_t i = 0; i < loopFunctions.FoodList.size(); i++) {
		x = loopFunctions.FoodList[i].GetX();
		y = loopFunctions.FoodList[i].GetY();
		DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, loopFunctions.FoodColoringList[i]);
	}
 
	 //draw food in nests
	 /*for (size_t i=0; i< loopFunctions.CollectedFoodList.size(); i++)
	 { 
	        x = loopFunctions.CollectedFoodList[i].GetX();
	        y = loopFunctions.CollectedFoodList[i].GetY();
	        DrawCylinder(CVector3(x, y, 0.002), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::BLACK);
	  } */ 
}

void CPFA_qt_user_functions::DrawFidelity() {

	   Real x, y;
        for(map<string, CVector2>::iterator it= loopFunctions.FidelityList.begin(); it!=loopFunctions.FidelityList.end(); ++it) {
            x = it->second.GetX();
            y = it->second.GetY();
            DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, CColor::YELLOW);
        }
}

void CPFA_qt_user_functions::DrawPheromones() {

	Real x, y, weight;
	vector<CVector2> trail;
	CColor trailColor = CColor::GREEN, pColor = CColor::GREEN;

	    for(size_t i = 0; i < loopFunctions.PheromoneList.size(); i++) {
		       x = loopFunctions.PheromoneList[i].GetLocation().GetX();
		       y = loopFunctions.PheromoneList[i].GetLocation().GetY();

		       if(loopFunctions.DrawTrails == 1) {
			          trail  = loopFunctions.PheromoneList[i].GetTrail();
			          weight = loopFunctions.PheromoneList[i].GetWeight();
                

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = trailColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = trailColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = trailColor = CColor::RED;
      
             CRay3 ray;
             size_t j = 0;
      
             for(j = 1; j < trail.size(); j++) {
                 ray = CRay3(CVector3(trail[j - 1].GetX(), trail[j - 1].GetY(), 0.01),
		CVector3(trail[j].GetX(), trail[j].GetY(), 0.01));
                 
                 DrawRay(ray, trailColor, 1.0);
             }

	 DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
		       } 
         else {
			          weight = loopFunctions.PheromoneList[i].GetWeight();

             if(weight > 0.25 && weight <= 1.0)        // [ 100.0% , 25.0% )
                 pColor = CColor::GREEN;
             else if(weight > 0.05 && weight <= 0.25)  // [  25.0% ,  5.0% )
                 pColor = CColor::YELLOW;
             else                                      // [   5.0% ,  0.0% ]
                 pColor = CColor::RED;
      
             DrawCylinder(CVector3(x, y, 0.0), CQuaternion(), loopFunctions.FoodRadius, 0.025, pColor);
         }
 }
}

void CPFA_qt_user_functions::DrawTargetRays() {
	//size_t tick = loopFunctions.GetSpace().GetSimulationClock();
	//size_t tock = loopFunctions.GetSimulator().GetPhysicsEngine("default").GetInverseSimulationClockTick() / 8;

	//if(tock == 0) tock = 1;

	//if(tick % tock == 0) {
		for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j], loopFunctions.TargetRayColorList[j]);
		}
	//}
}

/*
void CPFA_qt_user_functions::DrawTargetRays() {

	CColor c = CColor::BLUE;

	for(size_t j = 0; j < loopFunctions.TargetRayList.size(); j++) {
			DrawRay(loopFunctions.TargetRayList[j],c);
	}

	//if(loopFunctions.SimTime % (loopFunctions.TicksPerSecond * 10) == 0) {
		// comment out for DSA, uncomment for CPFA
		loopFunctions.TargetRayList.clear();
	//}
}
*/

void CPFA_qt_user_functions::DrawGrid() {
	// Get grid parameters from loop functions
	const size_t gridWidth = loopFunctions.GridWidth;
	const size_t gridHeight = loopFunctions.GridHeight;
	
	// Get arena bounds dynamically from simulator
	const argos::CVector3 arenaSize = CSimulator::GetInstance().GetSpace().GetArenaSize();
	const argos::Real arenaMinX = -arenaSize.GetX() / 2.0;
	const argos::Real arenaMaxX =  arenaSize.GetX() / 2.0;
	const argos::Real arenaMinY = -arenaSize.GetY() / 2.0;
	const argos::Real arenaMaxY =  arenaSize.GetY() / 2.0;
	const argos::Real arenaWidth = arenaMaxX - arenaMinX;
	const argos::Real arenaHeight = arenaMaxY - arenaMinY;
	
	// Calculate cell size
	const argos::Real cellSizeX = arenaWidth / gridWidth;
	const argos::Real cellSizeY = arenaHeight / gridHeight;
	
	// Draw vertical grid lines
	for(size_t i = 0; i <= gridWidth; i++) {
		argos::Real x = arenaMinX + i * cellSizeX;
		argos::CVector3 start(x, arenaMinY, 0.001);
		argos::CVector3 end(x, arenaMaxY, 0.001);
		argos::CRay3 gridLine(start, end);
		DrawRay(gridLine, CColor::RED, 0.5);
	}
	
	// Draw horizontal grid lines
	for(size_t j = 0; j <= gridHeight; j++) {
		argos::Real y = arenaMinY + j * cellSizeY;
		argos::CVector3 start(arenaMinX, y, 0.001);
		argos::CVector3 end(arenaMaxX, y, 0.001);
		argos::CRay3 gridLine(start, end);
		DrawRay(gridLine, CColor::RED, 0.5);
	}
	
	// Draw grid points (cell centers)
	// for(size_t i = 0; i < gridWidth; i++) {
	// 	for(size_t j = 0; j < gridHeight; j++) {
	// 		// Convert grid indices to world coordinates (center of each cell)
	// 		argos::Real worldX = arenaMinX + (i + 0.5) * cellSizeX;
	// 		argos::Real worldY = arenaMinY + (j + 0.5) * cellSizeY;
			
	// 		// Get visit count for this cell
	// 		argos::CVector2 cellCenter(worldX, worldY);
	// 		// int visitCount = loopFunctions.getGridVisitCount(cellCenter);
			
	// 		// Color based on visit count
	// 		CColor pointColor;
	// 		// if(visitCount == 0) {
	// 		// 	pointColor = CColor::BLUE;  // Unvisited
	// 		// } else if(visitCount <= 2) {
	// 		// 	pointColor = CColor::CYAN;  // Lightly visited
	// 		// } else if(visitCount <= 5) {
	// 		// 	pointColor = CColor::GREEN; // Moderately visited
	// 		// } else if(visitCount <= 10) {
	// 		// 	pointColor = CColor::YELLOW; // Heavily visited
	// 		// } else {
	// 		// 	pointColor = CColor::RED;   // Very heavily visited
	// 		// }
	// 		pointColor = CColor::RED;
	// 		// Draw small cylinder at grid point
	// 		DrawCylinder(CVector3(worldX, worldY, 0.002), CQuaternion(), 0.02, 0.01, pointColor);
	// 	}
	// }
}

REGISTER_QTOPENGL_USER_FUNCTIONS(CPFA_qt_user_functions, "CPFA_qt_user_functions")
