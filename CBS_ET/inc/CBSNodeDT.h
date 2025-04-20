#pragma once
#include "CBSNode.h"


class CBSNodeDT : public HLNode
{
public:
	size_t sum_of_costs = 0; // sum of travel times for all paths

	// the following is used to comapre nodes in the CLEANUP list
	struct compare_node_by_f 
	{
		bool operator()(const CBSNodeDT* n1, const CBSNodeDT* n2) const 
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->distance_to_go == n2->distance_to_go)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						if (n1->h_val  == n2->h_val)
						{
							return n1->sum_of_costs >= n2->sum_of_costs;
						}
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
				}
				return n1->distance_to_go >= n2->distance_to_go;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;

			// 以下注释掉的部分是选择不同的节点选择顺序
			/*if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->h_val == n2->h_val)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						return n1->distance_to_go >= n2->distance_to_go;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;	
				}
				return n1->h_val >= n2->h_val;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;*/
		}
	};  // used by CLEANUP to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to comapre nodes in the FOCAL list
	struct compare_node_by_d 
	{
		bool operator()(const CBSNodeDT* n1, const CBSNodeDT* n2) const 
		{
			if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
					{
						if (n1->h_val  == n2->h_val)
						{
							return n1->sum_of_costs >= n2->sum_of_costs;
						}
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->distance_to_go >= n2->distance_to_go;
			/*if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
				{
					if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
				}
				return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
			}
			return n1->distance_to_go >= n2->distance_to_go;*/
		}
	};  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)

	// the following is used to compare nodes in the OPEN list
	struct compare_node_by_inadmissible_f
	{
		bool operator()(const CBSNodeDT* n1, const CBSNodeDT* n2) const
		{
			if (n1->g_val + n1->cost_to_go == n2->g_val + n2->cost_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->distance_to_go == n2->distance_to_go)
					{
						if (n1->h_val  == n2->h_val)
						{
							return n1->sum_of_costs >= n2->sum_of_costs;
						}
						return n1->h_val >= n2->h_val;
					}
					return n1->distance_to_go >= n2->distance_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->g_val + n1->cost_to_go >= n2->g_val + n2->cost_to_go;
		}
	};  // used by FOCAL to compare nodes by num_of_collisions (top of the heap has min h-val)

	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_f> >::handle_type cleanup_handle;
	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_inadmissible_f> >::handle_type open_handle;
	pairing_heap< CBSNodeDT*, compare<CBSNodeDT::compare_node_by_d> >::handle_type focal_handle;

	CBSNodeDT* parent = nullptr;
	//高效的实现：这里不是存储所有路径，而是只存发生改变的agent的全部新路径。因此后续建立全体agent的paths的时候需要不断回溯其父节点，记录每个agent的最后一次更新
	list< pair< int, Path> > paths; // new paths.  <agent_id, Path>

	inline int getFHatVal() const override { return g_val + cost_to_go; }
	inline int getSOC() const { return sum_of_costs; }
	inline int getNumNewPaths() const override { return (int) paths.size(); }
	inline string getName() const override { return "CBSDT Node"; }
	list<int> getReplannedAgents() const override
	{
		list<int> rst;
		for (const auto& path : paths)
			rst.push_back(path.first);
		return rst;
	}
};