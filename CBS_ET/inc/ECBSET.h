#pragma once
#include "CBS.h"
#include "ECBSNodeDT.h"
#include "SpaceTimeAStarET.h"
#include "common.h"
#include "CBSDTHeuristic.h"


class ECBSET : public CBS
{
public:
	int obj = -1; //0:max ET; 1:total ET
	vector<int> due_times;
	vector<int> laxity_times; //记录agents的时间裕度
	CBSDTHeuristic heuristic_helper_dt;
	
	int solution_obj = -2; // due time related

	/////////////////////////////////////////////////////////////////////////////////////////
	// set params
	// enum heuristics_type { ZERO, CG, DG, WDG, GLOBAL, PATH, LOCAL, CONFLICT, STRATEGY_COUNT }
	// class CBSHeuristic
	void setHeuristicType(heuristics_type h, heuristics_type h_hat)
	{
	    heuristic_helper_dt.type = h;
	    heuristic_helper_dt.setInadmissibleHeuristics(h_hat);
	}
	
	void setPrioritizeConflicts(bool p) {PC = p;	heuristic_helper_dt.PC = p; }
	void setRectangleReasoning(bool r) {rectangle_reasoning = r; heuristic_helper_dt.rectangle_reasoning = r; }
	void setCorridorReasoning(bool c) {corridor_reasoning = c; heuristic_helper_dt.corridor_reasoning = c; }
	void setTargetReasoning(bool t) {target_reasoning = t; heuristic_helper_dt.target_reasoning = t; }
	void setMutexReasoning(bool m) {mutex_reasoning = m; heuristic_helper_dt.mutex_reasoning = m; }
	void setDisjointSplitting(bool d) {disjoint_splitting = d; heuristic_helper_dt.disjoint_splitting = d; }
	void setBypass(bool b) { bypass = b; } // 2-agent solver for heuristic calculation does not need bypass strategy.
	
	// enum conflict_selection {RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS};
	void setConflictSelectionRule(conflict_selection c) { conflict_selection_rule = c; heuristic_helper_dt.conflict_seletion_rule = c; }

	// enum node_selection { NODE_RANDOM, NODE_H, NODE_DEPTH, NODE_CONFLICTS, NODE_CONFLICTPAIRS, NODE_MVC };
	void setNodeSelectionRule(node_selection n) { node_selection_rule = n; heuristic_helper_dt.node_selection_rule = n; }
	void setSavingStats(bool s) { save_stats = s; heuristic_helper_dt.save_stats = s; }

	ECBSET(const Instance& instance, bool sipp, int screen, int objective) : CBS(instance, sipp, screen), due_times(instance.getDueTimes()), obj(objective), heuristic_helper_dt(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper, instance.getDueTimes(), objective) 
	{
		if (objective == 1)
		{
			obj_lowerbound = -MAX_COST;
			inadmissible_obj_lowerbound = -MAX_COST;
		}
		else
		{
			obj_lowerbound = 0;
			inadmissible_obj_lowerbound = 0;
		}

		search_engines.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SpaceTimeAStar(instance, i);

		search_engines_dt.resize(num_of_agents);
		laxity_times.resize(num_of_agents, -1);
		for (int i = 0; i < num_of_agents; i++)
			search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
	};

	ECBSET(const Instance& instance, const vector<ConstraintTable>& initial_constraints, bool sipp, int screen, int objective) : CBS(instance, initial_constraints, sipp, screen), due_times(instance.getDueTimes()), obj(objective), heuristic_helper_dt(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper, instance.getDueTimes(), objective)  
	{
		if (objective == 1)
		{
			obj_lowerbound = -MAX_COST;
			inadmissible_obj_lowerbound = -MAX_COST;
		}
		else
		{
			obj_lowerbound = 0;
			inadmissible_obj_lowerbound = 0;
		}

		search_engines.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SpaceTimeAStar(instance, i);

		search_engines_dt.resize(num_of_agents);
		laxity_times.resize(num_of_agents, -1);
		for (int i = 0; i < num_of_agents; i++)
			search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
	};
	
	~ECBSET();

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit, int _obj_lowerbound = 0);
	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;

    void clear(); // used for rapid random  restart

	void clearSearchEngines();
	void printPaths() const;

private:

	int obj_lowerbound;
	int obj_upperbound = MAX_COST;
	int inadmissible_obj_lowerbound;

	vector<int> min_f_vals; // lower bounds of the cost of the shortest path
	vector<int> min_travel_times;
	vector< pair<Path, int> > paths_found_initially;  // contain initial paths found

	// vector<MDD*> mdds_initially;  // contain initial paths found
	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' mdd

	vector < SingleAgentSolver* > search_engines_dt;  // used to find (single) agents' due time related paths

	inline void releaseNodes();

	ECBSNodeDT* goal_node;
	bool terminate(ECBSNodeDT* curr);

	// print and save
	void printResults() const;
	static void printConflicts(const HLNode &curr) ;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;
	string getSolverName() const;

private:
	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_f> > cleanup_list; // it is called open list in ECBS
	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_inadmissible_f> > open_list; // this is used for EES
	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_d> > focal_list; // this is ued for both ECBS and EES

	void adoptBypass(ECBSNodeDT* curr, ECBSNodeDT* child, const vector<int>& fmin_copy);

	// node operators
	void pushNode(ECBSNodeDT* node);
	ECBSNodeDT* selectNode();
	bool reinsertNode(ECBSNodeDT* node);

	 // high level search
	bool generateChild(ECBSNodeDT* child, ECBSNodeDT* curr);
	bool generateRoot();
	bool findPathForSingleAgent(ECBSNodeDT*  node, int ag);
	void classifyConflicts(ECBSNodeDT &node);
	void computeConflictPriority(shared_ptr<Conflict>& con, ECBSNodeDT& node);

	//update information
	void updatePaths(ECBSNodeDT* curr);
};