#pragma once
#include "CBS.h"
#include "CBSNodeDT.h"
#include "SpaceTimeAStarET.h"
#include "CBSDTHeuristic.h"

class CBSET : public CBS
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

	CBSET(const Instance& instance, bool sipp, int screen, int objective) : CBS(instance, sipp, screen), due_times(instance.getDueTimes()), obj(objective), heuristic_helper_dt(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper, instance.getDueTimes(), objective) 
	{
		search_engines.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SpaceTimeAStar(instance, i);

		search_engines_dt.resize(num_of_agents);
		laxity_times.resize(num_of_agents, -1);
		for (int i = 0; i < num_of_agents; i++)
			//search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
            search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
	};
	
	CBSET(const Instance& instance, const vector<ConstraintTable>& initial_constraints, bool sipp, int screen, int objective) : CBS(instance, initial_constraints, sipp, screen), due_times(instance.getDueTimes()), obj(objective) , heuristic_helper_dt(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper, instance.getDueTimes(), objective) 
	{
		search_engines.resize(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
			search_engines[i] = new SpaceTimeAStar(instance, i);
		
		search_engines_dt.resize(num_of_agents);
		laxity_times.resize(num_of_agents, -1);
		for (int i = 0; i < num_of_agents; i++)
			//search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
            search_engines_dt[i] = new SpaceTimeAStarET(instance, i);
	};
	
	~CBSET();

	////////////////////////////////////////////////////////////////////////////////////////////
	// Runs the algorithm until the problem is solved or time is exhausted 
	bool solve(double time_limit, int _obj_lowerbound = 0, int _obj_upperbound = MAX_COST);

	// Save results
	void saveResults(const string &fileName, const string &instanceName) const;
	
	void clear(); // used for rapid random  restart

	void clearSearchEngines();

    void splicePaths(vector<Path> &space_paths,vector<Path> &final_paths) const;

protected:

	int obj_lowerbound = 0;
	int obj_upperbound = MAX_COST;
	int inadmissible_obj_lowerbound;
	
	// vector<MDD*> mdds_initially;  // contain initial paths found
	vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' mdd

	vector < SpaceTimeAStarET* > search_engines_dt;  // used to find (single) agents' due time related paths

	inline void releaseNodes();

	CBSNodeDT* goal_node;
	bool terminate(CBSNodeDT* curr);

	// print and save
	void printResults() const;
	static void printConflicts(const HLNode &curr) ;

	bool validateSolution() const;
	inline int getAgentLocation(int agent_id, size_t timestep) const;
	string getSolverName() const;

private: // CBS only, cannot be used by ECBS
	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_f> > cleanup_list; // it is called open list in ECBS
	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_inadmissible_f> > open_list; // this is used for EES
	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_d> > focal_list; // this is used for both ECBS and EES

	// node operators
	void pushNode(CBSNodeDT* node);
	CBSNodeDT* selectNode();
	bool reinsertNode(CBSNodeDT* node);

	// high level search
	bool generateChild(CBSNodeDT* child, CBSNodeDT* curr);
	bool generateRoot();
	bool findPathForSingleAgent(CBSNodeDT*  node, int ag, int lower_bound = 0);
	void classifyConflicts(CBSNodeDT &node);
	void computeConflictPriority(shared_ptr<Conflict>& con, CBSNodeDT& node);
	
	//update information
	void updatePaths(CBSNodeDT* curr);
	
};