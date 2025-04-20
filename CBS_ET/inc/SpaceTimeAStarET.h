#pragma once
#include "SpaceTimeAStar.h"
#include <string>

class SpaceTimeAStarET: public SpaceTimeAStar
{
public:
	int due_time;

	int replan_times = 0;
	int replan_success_times = 0;
	uint64_t replan_expanded_num = 0;
	uint64_t replan_generated_num = 0;

    //如果一开始lower_bound>=due_time,退化成最短路径问题
    //过滤掉所有lower_bound到upper_bound之外的A*节点
    //保持lower_bound<=due_time<=upper_bound
    //实时维护最优路径current_best
    //注意每次popNode时，因为lower_bound和upper_bound更新了，可能需要放弃这个节点
    //如果求解到了目标点，设此时路径长度为new_cost
    //采用原来的A*启发式,给定lower_bound里求出的路径是最短的
    //如果new_cost>=due_time,返回该路径
    //否则,更新current_best,
    //lower_bound=new_cost+1,upper_bound=2*due_time-new_cost-1

    Path findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                                        const vector<Path*>& paths, int agent, int lowerbound)override;

    //min_f_val是和due time的差值
	pair<Path, int> findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lower_bound, double w) override;

	Path findOptimalPath(const unordered_set<int>&forbid_locations);

	Path findOptimalPath(const unordered_map<pair<int,int>,int> &edge_heat_map,const unordered_set<int>&forbid_locations);

	string getName() const override { return "AStarET"; }

	SpaceTimeAStarET(const Instance& instance, int agent):
		SpaceTimeAStar(instance, agent),due_time(instance.getDueTimes()[agent])
		{}

    ~SpaceTimeAStarET(){releaseNodes();}    

protected:

    inline void pushNode(AStarNode* node);

    inline AStarNode* popNode();

    void updateFocalList();

};
