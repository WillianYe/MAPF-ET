#pragma once
#include "CBSHeuristic.h"
#include "CBSNodeDT.h"
#include "ECBSNodeDT.h"


class CBSDTHeuristic:public CBSHeuristic
{
public:

	vector<int> due_times;
	int obj;

	CBSDTHeuristic(int num_of_agents,
							const vector<Path*>& paths,
							vector<SingleAgentSolver*>& search_engines,
							const vector<ConstraintTable>& initial_constraints,
							MDDTable& mdd_helper, const vector<int>& due_times, int& obj) : CBSHeuristic( num_of_agents, paths, search_engines, initial_constraints, mdd_helper), due_times(due_times), obj(obj)
							{};
	
	// for MAPF-DT
	bool computeInformedHeuristics(CBSNodeDT& curr, const vector<int>& laxity_times, double time_limit);
	bool computeInformedHeuristics(ECBSNodeDT& curr, const vector<int>& min_f_vals, const vector<int>& laxity_times, double time_limit);
	// runtime reduction techniques。通过lazy computation，首先快速计算节点的h。后面被扩展时才再对其进行复杂的启发式值计算
	void computeQuickHeuristics(HLNode& curr);

	// for EES
	void updateInadmissibleHeuristics(HLNode& curr);
	void updateOnlineHeuristicErrors(CBSNodeDT& curr);
	void updateOnlineHeuristicErrors(ECBSNodeDT& curr);
    

	// build graphs
	bool buildWeightedDependencyGraph(CBSNodeDT& node, vector<int>& WDG, const vector<int>& laxity_times);
	bool buildWeightedDependencyGraph(ECBSNodeDT& node, vector<int>& WDG, const vector<int>& min_f_vals, const vector<int>& laxity_times, int& delta_g);

	// 利用MDD是否相交来判定是否依赖
	bool dependent(int a1, int a2, HLNode& node);
	static bool SyncMDDs(const MDD &mdd1, const MDD& mdd2); 	// Match and prune MDD according to another MDD.
	
	// return h value and num of CT nodes
	pair<int, int> solve2Agents(int a1, int a2, const CBSNodeDT& node, bool cardinal); 
    tuple<int, int, int> solve2Agents(int a1, int a2, const ECBSNodeDT& node);

    static int greedyWeightedMatching(const vector<int>& WDG, int cols);
	int minimumWeightedVertexCover(const vector<int>& WDG);
	int weightedVertexCover(const vector<int>& WDG);
	int DPForWMVC(vector<int>& x, int i, int sum, const vector<int>& WDG, const vector<int>& range, int& best_so_far); // dynamic programming
	int ILPForConstrainedWMVC(const std::vector<int>& WDG, const std::vector<int>& range);
	int DPForConstrainedWMVC(vector<bool>& x, int i, int sum, const vector<int>& WDG, const vector<int>& range, int& best_so_far);
};




