#pragma once
#include "CBSNode.h"


class ECBSNodeDT : public HLNode
{
public:
	int sum_of_costs = 0; // sum of travel times for all paths
	int collective_obj = 0; // sum of due time related objective values

	// the following is used to comapre nodes in the CLEANUP list
	struct compare_node_by_f
	{
		bool operator()(const ECBSNodeDT* n1, const ECBSNodeDT* n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->distance_to_go == n2->distance_to_go)
				{
					if (n1->collective_obj + n1->cost_to_go == n2->collective_obj + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->collective_obj + n1->cost_to_go >= n2->collective_obj + n2->cost_to_go;
				}
				return n1->distance_to_go >= n2->distance_to_go;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by CLEANUP to compare nodes by f_val (top of the heap has min f_val)

	// the following is used to comapre nodes in the FOCAL list
	struct compare_node_by_d
	{
		bool operator()(const ECBSNodeDT* n1, const ECBSNodeDT* n2) const
		{
			if (n1->distance_to_go == n2->distance_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->collective_obj + n1->cost_to_go == n2->collective_obj + n2->cost_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->collective_obj + n1->cost_to_go >= n2->collective_obj + n2->cost_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->distance_to_go >= n2->distance_to_go;
		}
	};  // used by FOCAL to compare nodes by distance_to_go (top of the heap has min distance_to_go)

		// the following is used to compare nodes in the OPEN list
	struct compare_node_by_inadmissible_f
	{
		bool operator()(const ECBSNodeDT* n1, const ECBSNodeDT* n2) const
		{
			if (n1->collective_obj + n1->cost_to_go == n2->collective_obj + n2->cost_to_go)
			{
				if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
				{
					if (n1->distance_to_go == n2->distance_to_go)
					{
						return n1->h_val >= n2->h_val;
					}
					return n1->distance_to_go >= n2->distance_to_go;
				}
				return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
			}
			return n1->collective_obj + n1->cost_to_go >= n2->collective_obj + n2->cost_to_go;
		}
	};  // used by FOCAL to compare nodes by f^-val (top of the heap has min f^-val)

	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_f> >::handle_type cleanup_handle;
	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_inadmissible_f> >::handle_type open_handle;
	pairing_heap< ECBSNodeDT*, compare<ECBSNodeDT::compare_node_by_d> >::handle_type focal_handle;

	ECBSNodeDT* parent = nullptr;
	list< pair< int, pair<Path, int> > > paths; // new paths <agent id, <path, min f>>	
	//注意：f_hat和f的区别
	inline int getFHatVal() const { return collective_obj + cost_to_go; }
	inline int getNumNewPaths() const { return (int) paths.size(); }
	inline string getName() const { return "ECBSDT Node"; }
	list<int> getReplannedAgents() const
	{
		list<int> rst;
		for (const auto& path : paths)
			rst.push_back(path.first);
		return rst;
	}
};