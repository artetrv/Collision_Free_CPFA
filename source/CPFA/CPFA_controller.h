#ifndef CPFA_CONTROLLER_H
#define CPFA_CONTROLLER_H

#include <source/Base/BaseController.h>
#include <source/Base/Pheromone.h>
#include <source/CPFA/CPFA_loop_functions.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
//#include <cmath>
#include <deque>

using namespace std;
using namespace argos;

static unsigned int num_targets_collected = 0;

class CPFA_loop_functions;

class CPFA_controller : public BaseController {

	public:

		CPFA_controller();

		// CCI_Controller inheritence functions
		void Init(argos::TConfigurationNode &node);
		void ControlStep();
		void Reset();

		bool IsHoldingFood();
		bool IsUsingSiteFidelity();
		bool IsInTheNest();
		argos::Real getSimTimeInSeconds();

		Real FoodDistanceTolerance;

		void SetLoopFunctions(CPFA_loop_functions* lf);
  
  size_t     GetSearchingTime();//qilu 09/26/2016
  size_t      GetTravelingTime();//qilu 09/26/2016
  string      GetStatus();//qilu 09/26/2016
  size_t      startTime;//qilu 09/26/2016
  
  Real curr_time_in_seconds; 
    Real last_time_in_seconds; 
        

	private:

	 // ===== Congestion detection (sliding-window tortuosity) =====
    size_t      s_WindowSize;     // window length 
    size_t      sw_sample_pos;  // sample every (TPS / divisor) ticks
    size_t      sw_waitTicks;    // derived: ticks between samples
    argos::Real sw_CongRatioOn;        // tau threshold to ENTER congested
    size_t      sw_bad_samples;            // consecutive "bad" samples to enter
    argos::Real sw_congEps;            // epsilon to guard tiny euclid distances


    // Runtime bookkeeping
    std::deque<argos::CVector2> sw_positions;  // sliding window of positions
    argos::Real   sum_window_segments;         // total distance traveled along the window (sum of segment lengths)
    size_t        sw_LastCongSampleTick;       // Stores the tick when the last sample was taken
    size_t        sw_badSample_counter;            // hysteresis counters: Counts how many consecutive samples had a high tortuosity
    bool          InCongested;              // state flag

	 // Cooldown after congestion-drop: forbid pickups until this tick
	size_t        cooldownUntilTick;
  	size_t        cooldownTicks;            // derived from CongDropCooldownSec * TPS

    // ===== Congestion detection (sliding-window tortuosity) END =====

  string 			controllerID;//qilu 07/26/2016

		CPFA_loop_functions* LoopFunctions;
		argos::CRandom::CRNG* RNG;

		/* pheromone trail variables */
		std::vector<argos::CVector2> TrailToShare;
		std::vector<argos::CVector2> TrailToFollow;
		std::vector<argos::CRay3>    MyTrail;

		/* robot position variables */
		argos::CVector2 SiteFidelityPosition;
  bool			 updateFidelity; //qilu 09/07/2016
  
		vector<CRay3> myTrail;
		CColor        TrailColor;

		bool isInformed;
		bool isHoldingFood;
		bool isUsingSiteFidelity;
		bool isGivingUpSearch;
  
		size_t ResourceDensity;
		size_t RobotDensity; //qilu 06/2023
		size_t MaxTrailSize;
		size_t SearchTime;//for informed search
  
  size_t           searchingTime; //qilu 09/26
  size_t           travelingTime;//qilu 09/26
        
  
		/* iAnt CPFA state variable */
		enum CPFA_state {
			DEPARTING = 0,
			SEARCHING = 1,
			RETURNING = 2,
			SURVEYING = 3,
			CONGESTED = 4
		} CPFA_state;

		/* iAnt CPFA state functions */
		void CPFA();
		void Departing();
		void Searching();
		void Returning();
		void Surveying();
		void Congested();

		// congestion helpers
		void Cong_ResetWindow(); // clears window
		void Cong_TrySampleAndUpdate();    /*runs when returning, samples every few ticks, adds new path segment, maintains fixed-size deque
		waits until window has enough samples(100), computes tortuosity ratio, updates hysteresis counters, calls cong_enter/exit*/
		bool Cong_WindowFull() const;  //true when sw is full
		argos::Real Cong_CurrentTortuosity() const; //computes (total path lenght/ straight line distance between oldest and newest positions) return 1 when insufficient data
		void Cong_Enter(); //marks the start of congestion

		/* CPFA helper functions */
		void SetRandomSearchLocation();
		void SetHoldingFood();
		void SetLocalResourceDensity();
		void SetRobotDensity(); //qilu 06/2023
		
		void SetFidelityList(argos::CVector2 newFidelity);
		void SetFidelityList();
		bool SetTargetPheromone();

		argos::Real GetExponentialDecay(argos::Real value, argos::Real time, argos::Real lambda);
		argos::Real GetBound(argos::Real value, argos::Real min, argos::Real max);
		argos::Real GetPoissonCDF(argos::Real k, argos::Real lambda);

		void UpdateTargetRayList();
  
		CVector2 previous_position;

		string results_path;
		string results_full_path;
		bool isUsingPheromone;

		unsigned int survey_count;
		/* Pointer to the LEDs actuator */
        CCI_LEDsActuator* m_pcLEDs;
};

#endif /* CPFA_CONTROLLER_H */
