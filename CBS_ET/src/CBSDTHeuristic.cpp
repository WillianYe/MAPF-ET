//#pragma warning(disable: 4996) //Jiaoyang: I added this line to disable error C4996 caused by CPLEX
#include "CBSDTHeuristic.h"
#include "CBS.h"
#include <algorithm>
#include <queue>
//#include <ilcplex/ilocplex.h>


void CBSDTHeuristic::updateInadmissibleHeuristics(HLNode& curr)
{
	int h = curr.getName() == "CBS Node"? curr.h_val : 0;
	double cost_error = 0, distance_error = 0;
	double c;
	switch (inadmissible_heuristic)
	{
	case heuristics_type::PATH:
		// update errors along the path to the node
		num_of_errors[0] = 0;
		sum_distance_errors[0] = 0;
		sum_cost_errors[0] = 0;
		for (auto ptr = curr.parent; ptr  != nullptr; ptr = ptr->parent)
		{
			if (ptr->fully_expanded)
			{
				num_of_errors[0]++;
				sum_distance_errors[0] += ptr->distance_error;
				sum_cost_errors[0] += ptr->cost_error;
			}
		}
	case heuristics_type::GLOBAL: // Note: there is no "break" in the previous line, so here we compute heuristics for both GLOBAL and PATH
		if (num_of_errors[0] < 1)
		{
			curr.cost_to_go = max(0, curr.getFVal() - curr.getFHatVal()); // ensure that f <= f^
			return;
		}
		if (num_of_errors[0] <= sum_distance_errors[0])
		{
			c =  sum_cost_errors[0] / num_of_errors[0] * 10;
		}
		else
		{
			c = sum_cost_errors[0] / (num_of_errors[0] - sum_distance_errors[0]);
		}
		curr.cost_to_go = h + (int)(curr.distance_to_go * c);
		break;
	case heuristics_type::LOCAL:
		if (std::abs(1 - sum_distance_errors[0]) < 0.001)
			curr.cost_to_go = h + max(0, (int)(curr.distance_to_go) * 1000);
		else
			curr.cost_to_go = h + max(0, (int)(curr.distance_to_go * sum_cost_errors[0] / (1 - sum_distance_errors[0])));
		/*if (num_of_errors % 100 == 0)
		cout << std::setprecision(3)
		<< sum_cost_error  << ","
		<< sum_distance_error << ","
		<< sum_cost_error  / (1 - sum_distance_error) << endl;*/
		break;
	case heuristics_type::CONFLICT:
		if (curr.conflicts.empty() && curr.unknownConf.empty())
			return;
		for (const auto& conflict : curr.conflicts)
		{
			int id = conflict->getConflictId();
			cost_error += getCostError(id);
			distance_error += getDistanceError(id);
		}

		/*set<pair<int, int>> conflicting_agents;
		for (const auto& conflict : curr.unknownConf)
		{
			auto agents = make_pair(min(conflict->a1, conflict->a2), max(conflict->a1, conflict->a2));
			if (conflicting_agents.find(agents) != conflicting_agents.end())
				continue; // we do not recount the conflict between the same pair of agents
			int id = conflict->getConflictId();
			cost_error += getCostError(id);
			distance_error += getDistanceError(id);
		}*/
		cost_error /= (double)(curr.conflicts.size()); // + conflicting_agents.size());
		distance_error /= (double)(curr.conflicts.size()); // + conflicting_agents.size());
		// curr.cost_to_go = h + (int)(curr.distance_to_go * cost_error);
		if ( distance_error >= 1)
			curr.cost_to_go = (int)(curr.distance_to_go * cost_error);
		else
			curr.cost_to_go = (int)(curr.distance_to_go * cost_error / (1 - distance_error));
		// cout << std::setprecision(3) << (double)curr.cost_to_go / curr.distance_to_go << ",";
		curr.cost_to_go = max(min(MAX_COST, curr.cost_to_go), 0);
		curr.cost_to_go += h;
		break;
	default:
		break;
	}
    if (curr.getFVal() > curr.getFHatVal())
        curr.cost_to_go += curr.getFVal() - curr.getFHatVal(); // ensure that f <= f^
}

void CBSDTHeuristic::updateOnlineHeuristicErrors(CBSNodeDT& curr)
{
    if ((inadmissible_heuristic == heuristics_type::GLOBAL ||
         inadmissible_heuristic == heuristics_type::PATH ||
         inadmissible_heuristic == heuristics_type::LOCAL ||
         inadmissible_heuristic == heuristics_type::CONFLICT) && curr.parent != nullptr)
	{
		curr.parent->fully_expanded = true;
		HLNode* best = &curr;
		for (auto child : curr.parent->children)
		{
			if (!child->h_computed)
			{
				curr.parent->fully_expanded = false;
				break;
			}
			if (best->getFVal() > child->getFVal() ||
				(best->getFVal() == child->getFVal() && best->distance_to_go > child->distance_to_go))
				best = child;
		}
		if (curr.parent->fully_expanded) // update error
		{
			curr.parent->distance_error = 1 + best->distance_to_go - curr.parent->distance_to_go;
			curr.parent->cost_error = best->g_val + best->h_val - curr.parent->g_val - curr.parent->h_val;
			if (inadmissible_heuristic == heuristics_type::GLOBAL) // update global error
			{
				sum_distance_errors[0] += curr.parent->distance_error;
				sum_cost_errors[0] += curr.parent->cost_error;
				num_of_errors[0]++;
			}
			else if (inadmissible_heuristic == heuristics_type::LOCAL) // update local error
			{
				num_of_errors[0]++;
				double learning_rate = 0.001;
				if (num_of_errors[0] * learning_rate < 1)
				{
					learning_rate = 1.0 / num_of_errors[0];
				}
				sum_distance_errors[0] = sum_distance_errors[0] *(1 - learning_rate) + curr.parent->distance_error * learning_rate;
				sum_cost_errors[0] = sum_cost_errors[0] * (1 - learning_rate) + curr.parent->cost_error * learning_rate;
			}
			else if (inadmissible_heuristic == heuristics_type::CONFLICT) // update conflict error
            {
			    int id = curr.parent->conflict->getConflictId();
                sum_distance_errors[id] += curr.parent->distance_error;
                sum_cost_errors[id] += curr.parent->cost_error;
                num_of_errors[id]++;
            }
		}
	}
}

void CBSDTHeuristic::updateOnlineHeuristicErrors(ECBSNodeDT& parent)
{
    if (inadmissible_heuristic == heuristics_type::ZERO)
        return;
	// Find the best child
	const HLNode* best_child = parent.children.front();
	assert(parent.children.size() <= 2);
	if (parent.children.size() == 2)
	{
		const HLNode* other = parent.children.back();
		if (best_child->getFHatVal() > other->getFHatVal() ||
			(best_child->getFHatVal() == other->getFHatVal() && best_child->distance_to_go > other->distance_to_go))
			best_child = other;
	}
    // Update the errors
    parent.distance_error = 1 + best_child->distance_to_go - parent.distance_to_go;
    parent.cost_error = best_child->getFHatVal() - best_child->cost_to_go  - parent.sum_of_costs;
    if (inadmissible_heuristic == heuristics_type::GLOBAL) // update global error
    {
        sum_distance_errors[0] += parent.distance_error;
        sum_cost_errors[0] += parent.cost_error;
        num_of_errors[0]++;
    }
    else if (inadmissible_heuristic == heuristics_type::LOCAL) // update local error
    {
        num_of_errors[0]++;
        double learning_rate = 0.001;
        if (num_of_errors[0] * learning_rate < 1)
        {
            learning_rate = 1.0 / num_of_errors[0];
        }
        sum_distance_errors[0] = sum_distance_errors[0] *(1 - learning_rate) + parent.distance_error * learning_rate;
        sum_cost_errors[0] = sum_cost_errors[0] * (1 - learning_rate) + parent.cost_error * learning_rate;
    }
    else if (inadmissible_heuristic == heuristics_type::CONFLICT) // update conflict error
    {
        int id = parent.conflict->getConflictId();
        sum_cost_errors[id] += parent.cost_error;
        num_of_errors[id]++;
        sum_distance_errors[id] += parent.distance_error;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////

// 节点按照h1从OPEN中pop之后，计算h2，并REINSERT压入OPEN
void CBSDTHeuristic::computeQuickHeuristics(HLNode& node)
{
	if (node.parent != nullptr)
		// 此处和论文中的公式不同，缺少一项。???
	    node.h_val = max(0, node.parent->g_val + node.parent->h_val - node.g_val); // pathmax

	// for EES
	node.updateDistanceToGo();
    if (node.parent != nullptr)
	    updateInadmissibleHeuristics(node); // compute inadmissible heuristics
}

// CBS-DT 主流程
bool CBSDTHeuristic::computeInformedHeuristics(CBSNodeDT& curr,const vector<int>& laxity_times, double _time_limit)
{
    curr.h_computed = true;
	// create conflict graph
	start_time = clock();
	this->time_limit = _time_limit;
	int num_of_CGedges;
	vector<int> WDG(num_of_agents * num_of_agents, 0); // heuristic graph
	int h = -1;

	// compute admissible heuristics
	switch (type)
	{
	case heuristics_type::ZERO:
		h = 0;
		break;
	case heuristics_type::WDG:
		if (!buildWeightedDependencyGraph(curr, WDG, laxity_times))
			return false;
		h = minimumWeightedVertexCover(WDG);
		break;
	default:
		break;
	}
	if (h < 0)
		return false;
	curr.h_val = max(h, curr.h_val);
	return true;
}

// ECBS-DT 主流程
bool CBSDTHeuristic::computeInformedHeuristics(ECBSNodeDT& curr, const vector<int>& min_f_vals, const vector<int>& laxity_times, double _time_limit)
{
    curr.h_computed = true;
	// create conflict graph
	start_time = clock();
	this->time_limit = _time_limit;
	int num_of_CGedges;
	vector<int> WDG(num_of_agents * num_of_agents, 0); // heuristic graph
	int h = -1;

	/* compute admissible heuristics */
	switch (type)
	{
	case heuristics_type::ZERO:
		h = 0;
		break;
	case heuristics_type::WDG:
	    int delta_g;
		if (!buildWeightedDependencyGraph(curr,WDG, min_f_vals,laxity_times, delta_g))
			return false;
		assert(delta_g >= 0);
		// cout << curr.g_val << "+" << delta_g << endl;
		// 暂时不用太复杂的启发函数，不考虑opt和min之间的差值
		// h = minimumWeightedVertexCover(WDG) + delta_g;
		h = minimumWeightedVertexCover(WDG);
		break;

	default:
		cerr << "ERROR in computing informed heuristics" << endl;
	}
	if (h < 0)
		return false;
	curr.h_val = max(h, curr.h_val);
    curr.cost_to_go = max(curr.cost_to_go, curr.getFVal() - curr.sum_of_costs); // ensure that f <= f^

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


// WDG for CBS-DT
bool CBSDTHeuristic::buildWeightedDependencyGraph(CBSNodeDT& node, vector<int>& WDG, const vector<int>& laxity_times)
{
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));

		int val = 0;
		// 记忆化，减少计算量
		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;
			// <h value, num of CT nodes, 0> for CBS
			// <h value, a1 f at root, a2 f at root> for ECBS
			val = get<0>(got->second);
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];
		}
		else if (rectangle_reasoning)   // 矩形对称不是cardinal
		{
			auto rst = solve2Agents(a1, a2, node, false);
			assert(rst.first >= 0);
			// <h value, num of CT nodes, 0> for CBS
			lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = make_tuple(rst.first, rst.second, 1);
			val = rst.first;
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];
		}
		else
		{
			bool depend = conflict->priority == conflict_priority::CARDINAL;
			//若存在cardinal冲突，则肯定为dependent
			// 若不是cardinal，则首先通过合并MDD判断是否dependent
			if (!depend && !mutex_reasoning) 
			{
				depend = dependent(a1, a2, node); // using merging MDD methods before runing 2-agent instance
			}

			if (depend) // run 2-agent solver only for dependent agents
			{
				auto rst = solve2Agents(a1, a2, node, depend);
				assert(rst.first >= 1); //依赖的权重至少为1
				lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = make_tuple(rst.first, rst.second, 1);
				val = rst.first;
				WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
				WDG[a2 * num_of_agents + a1] = WDG[idx];
			}
			else
			{
				lookupTable[a1][a2][HTableEntry(a1, a2, &node)]  = make_tuple(0, 1, 0); // h=0, #CT nodes = 1
				WDG[idx] = 0;
				WDG[a2 * num_of_agents + a1] = 0;
			}
		}
		
		if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
		{
			runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
			return false;
		}
		if (WDG[idx] == MAX_COST) // no solution
		{
            return false;
		}

		if (conflict->priority != conflict_priority::CARDINAL && val > 0)
			conflict->priority = conflict_priority::PSEUDO_CARDINAL; // the two agents are dependent, although resolving this conflict might not increase the cost

		// for MAPF-DT。将CARDINAL/PSEUDO_CARDINAL根据启发式信息细分为三类
		if (WDG[idx] > 0)
			conflict->priority = conflict_priority::DOUBLE_CARDINAL; 		
		else if (WDG[idx] == 0 && val > 0 && max(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2])) > 0)
			conflict->priority = conflict_priority::SINGLE_CARDINAL; 
		else if (WDG[idx] == 0 && val > 0 && max(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2])) == 0)
			conflict->priority = conflict_priority::LATENT_CARDINAL; 
	}

	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}

// WDG for ECBS-DT
bool CBSDTHeuristic::buildWeightedDependencyGraph(ECBSNodeDT& node, vector<int>& WDG, const vector<int>& min_f_vals, const vector<int>& laxity_times,  int& delta_g)
{
	// the sum of the difference between f_opt^i and f_min^i is delta_g
    delta_g = 0;
    vector<bool> counted(num_of_agents, false); // record the agents whose delta_g has been counted
	for (const auto& conflict : node.conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2);
		int a2 = max(conflict->a1, conflict->a2);
		int idx = a1 * num_of_agents + a2;
		auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		int val = 0;

		if (got != lookupTable[a1][a2].end()) // check the lookup table first
		{
			num_memoization++;

			// <h value, a1 f at root, a2 f at root> for ECBS
			val = get<0>(got->second);
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];

            if (!counted[a1])
            {
                assert(get<1>(got->second) >= min_f_vals[a1]);
				int temp = min(max(0, val+get<1>(got->second) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<1>(got->second) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(got->second) >= min_f_vals[a1]);
				int temp = min(max(0, val+get<2>(got->second) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<2>(got->second) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
		}
		else
		{
			auto rst = solve2Agents(a1, a2, node);
            lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = rst;
            if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
            {
                runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
                return false;
            }

			val = get<0>(rst);
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];

            if (!counted[a1])
            {
                assert(get<1>(rst) >= min_f_vals[a1]);
                int temp = min(max(0, val+get<1>(rst) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<1>(rst) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(rst) >= min_f_vals[a2]);
                int temp = min(max(0, val+get<2>(rst) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<2>(rst) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
		}

        if (WDG[idx] == MAX_COST) // no solution
            return false;
	}

    for (const auto& conflict : node.unknownConf)
    {
        int a1 = min(conflict->a1, conflict->a2);
        int a2 = max(conflict->a1, conflict->a2);
        int idx = a1 * num_of_agents + a2;
        auto got = lookupTable[a1][a2].find(HTableEntry(a1, a2, &node));
		int val = 0;

        if (got != lookupTable[a1][a2].end()) // check the lookup table first
        {
            num_memoization++;
            
			val = get<0>(got->second);
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];

            if (!counted[a1])
            {
                assert(get<1>(got->second) >= min_f_vals[a1]);
				int temp = min(max(0, val+get<1>(got->second) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<1>(got->second) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(got->second) >= min_f_vals[a1]);
				int temp = min(max(0, val+get<2>(got->second) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<2>(got->second) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
        }
        else
        {
            auto rst = solve2Agents(a1, a2, node);
            lookupTable[a1][a2][HTableEntry(a1, a2, &node)] = rst;
            if ((clock() - start_time) / CLOCKS_PER_SEC > time_limit) // run out of time
            {
                runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
                return false;
            }
            
			val = get<0>(rst);
			WDG[idx] = min(max(0, val-laxity_times[a1]),max(0, val-laxity_times[a2]));
            WDG[a2 * num_of_agents + a1] = WDG[idx];

            if (!counted[a1])
            {
                assert(get<1>(rst) >= min_f_vals[a1]);
                int temp = min(max(0, val+get<1>(rst) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<1>(rst) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
            if (!counted[a2])
            {
                assert(get<2>(rst) >= min_f_vals[a2]);
                int temp = min(max(0, val+get<2>(rst) - min_f_vals[a1]-laxity_times[a1]),max(0, val+get<2>(rst) - min_f_vals[a1]-laxity_times[a2]));
                delta_g += temp - WDG[idx];
                counted[a1] = true;
            }
        }

        if (WDG[idx] == MAX_COST) // no solution
            return false;
    }
	runtime_build_dependency_graph += (double)(clock() - start_time) / CLOCKS_PER_SEC;
	return true;
}

//通过两个AGENT的MDD的并判断是否存在依赖关系，与父类相同
bool CBSDTHeuristic::dependent(int a1, int a2, HLNode& node) // return true if the two agents are dependent
{
	const MDD* mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size()); // get mdds
	const MDD* mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());
	// 第二个MDD图高于第一个MDD
	if (mdd1->levels.size() > mdd2->levels.size()) // swap
		std::swap(mdd1, mdd2);
	num_merge_MDDs++;
	return !SyncMDDs(*mdd1, *mdd2);
}

// return true if the joint MDD exists.
bool CBSDTHeuristic::SyncMDDs(const MDD &mdd, const MDD& other) // assume mdd.levels <= other.levels
{
	if (other.levels.size() <= 1) // Either of the MDDs was already completely pruned already
		return false;

	SyncMDD dup(mdd);
	if (dup.levels.size() < other.levels.size())
	{
		size_t i = dup.levels.size();
		dup.levels.resize(other.levels.size());
		for (; i < dup.levels.size(); i++)
		{
			SyncMDDNode* parent = dup.levels[i - 1].front();
			auto node = new SyncMDDNode(parent->location, parent);
			parent->children.push_back(node);
			dup.levels[i].push_back(node);

		}
	}
	// Cheaply find the coexisting nodes on level zero - all nodes coexist because agent starting points never collide
	dup.levels[0].front()->coexistingNodesFromOtherMdds.push_back(other.levels[0].front());

	// what if level.size() = 1?
	for (size_t i = 1; i < dup.levels.size(); i++)
	{
		for (auto node = dup.levels[i].begin(); node != dup.levels[i].end();)
		{
			// Go over all the node's parents and test their coexisting nodes' children for co-existance with this node
			for (auto parent = (*node)->parents.begin(); parent != (*node)->parents.end(); parent++)
			{
				//bool validParent = false;
				for (const MDDNode* parentCoexistingNode : (*parent)->coexistingNodesFromOtherMdds)
				{
					for (const MDDNode* childOfParentCoexistingNode : parentCoexistingNode->children)
					{
						if ((*node)->location == childOfParentCoexistingNode->location ||// vertex conflict
							((*node)->location == parentCoexistingNode->location && (*parent)->location == childOfParentCoexistingNode->location)) // edge conflict
							continue;

						auto it = (*node)->coexistingNodesFromOtherMdds.cbegin();
						for (; it != (*node)->coexistingNodesFromOtherMdds.cend(); ++it)
						{
							if (*it == childOfParentCoexistingNode)
								break;
						}
						if (it == (*node)->coexistingNodesFromOtherMdds.cend())
						{
							(*node)->coexistingNodesFromOtherMdds.push_back(childOfParentCoexistingNode);
						}
					}
				}
			}
			if ((*node)->coexistingNodesFromOtherMdds.empty())
			{
				// delete the node, and continue up the levels if necessary
				SyncMDDNode* p = *node;
				node++;
				dup.deleteNode(p, i);
			}
			else
				node++;
		}
		if (dup.levels[i].empty())
		{
			dup.clear();
			return false;
		}
	}
	dup.clear();
	return true;
}


// 参考f-cardinal论文，找出f-cardinal的节点准则
// return optimal f - root g and #HL nodes
pair<int, int> CBSDTHeuristic::solve2Agents(int a1, int a2, const CBSNodeDT& node, bool cardinal)
{
	vector<SingleAgentSolver*> engines{search_engines[a1],   search_engines[a2]};
	vector<vector<int>> initial_paths{*paths[a1], *paths[a2]};
	vector<ConstraintTable> constraints{ConstraintTable(initial_constraints[a1]), ConstraintTable(initial_constraints[a2]) };
    constraints[0].insert2CT(node, a1);
    constraints[1].insert2CT(node, a2);
	CBS cbs(engines, constraints, initial_paths, screen);
	// setUpSubSolver(cbs);
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG, heuristics_type::ZERO);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1); // solve the sub problem optimally
	cbs.setNodeLimit(node_limit);

	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	int root_g = (int)initial_paths[0].size() - 1 + (int)initial_paths[1].size() - 1;
	int lowerbound = root_g;
	int upperbound = MAX_COST;
	if (cardinal)
		lowerbound += 1;
	cbs.solve(time_limit - runtime, lowerbound, upperbound);
	num_solve_2agent_problems++;

	pair<int, int> rst;
	if (cbs.runtime >= time_limit - runtime || cbs.num_HL_expanded > node_limit) // time out or node out
		rst.first = cbs.getLowerBound() - root_g; // using lowerbound to approximate
	else if (cbs.solution_cost  < 0) // no solution
		rst.first = MAX_COST;
	else
	{
		assert(cbs.solution_cost >= root_g);
		rst.first = cbs.solution_cost - root_g;
	}
	rst.second = (int)cbs.num_HL_expanded;
	// For statistic study!!!
	if (save_stats)
	{
		sub_instances.emplace_back(a1, a2, &node, cbs.num_HL_expanded, rst.second);
	}
	return rst;
}

// return optimal f and a1_shortestpath * #agents + a2_shortestpath
tuple<int, int, int> CBSDTHeuristic::solve2Agents(int a1, int a2, const ECBSNodeDT& node)
{
	vector<SingleAgentSolver*> engines{ search_engines[a1],   search_engines[a2] };
	vector<vector<int>> initial_paths;
	vector<ConstraintTable> constraints{ ConstraintTable(initial_constraints[a1]), ConstraintTable(initial_constraints[a2]) };
    constraints[0].insert2CT(node, a1);
    constraints[1].insert2CT(node, a2);
	CBS cbs(engines, constraints, initial_paths, screen);
	// setUpSubSolver(cbs);
	cbs.setPrioritizeConflicts(PC);
	cbs.setHeuristicType(heuristics_type::CG, heuristics_type::ZERO);
	cbs.setDisjointSplitting(disjoint_splitting);
	cbs.setBypass(false); // I guess that bypassing does not help two-agent path finding???
	cbs.setRectangleReasoning(rectangle_reasoning);
	cbs.setCorridorReasoning(corridor_reasoning);
	cbs.setTargetReasoning(target_reasoning);
	cbs.setMutexReasoning(mutex_reasoning);
	cbs.setConflictSelectionRule(conflict_seletion_rule);
	cbs.setNodeSelectionRule(node_selection_rule);
	cbs.setHighLevelSolver(high_level_solver_type::ASTAR, 1); // solve the sub problem optimally
	cbs.setNodeLimit(node_limit);

	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	cbs.solve(time_limit - runtime, 0, MAX_COST);
	num_solve_2agent_problems++;
	
	// For statistic study!!!
	if (save_stats)
	{
		sub_instances.emplace_back(a1, a2, &node, cbs.num_HL_expanded, (int)cbs.num_HL_expanded);
	}

	if (cbs.runtime >= time_limit - runtime) // time out
		return make_tuple(0, 0, 0); // using lowerbound to approximate
    else if (cbs.num_HL_expanded > node_limit) // node out
		return make_tuple(cbs.getLowerBound() - cbs.dummy_start->g_val,
		        cbs.getInitialPathLength(0), cbs.getInitialPathLength(1)); // using lowerbound to approximate
	else if (cbs.solution_cost  < 0) // no solution
		return make_tuple(MAX_COST, cbs.getInitialPathLength(0), cbs.getInitialPathLength(1));
	else
		return make_tuple(cbs.solution_cost - cbs.dummy_start->g_val,
		        cbs.getInitialPathLength(0), cbs.getInitialPathLength(1));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

int CBSDTHeuristic::minimumWeightedVertexCover(const vector<int>& WDG)
{
	clock_t t = clock();
	int rst = weightedVertexCover(WDG);
	num_solve_MVC++;
	runtime_solve_MVC += (double)(clock() - t) / CLOCKS_PER_SEC;
	return rst;
}

// branch and bound
// enumerate all possible assignments and return the best one
int CBSDTHeuristic::weightedVertexCover(const std::vector<int>& WDG)
{
	int rst = 0;
	std::vector<bool> done(num_of_agents, false);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (done[i])
			continue;
		std::vector<int> range;
		std::vector<int> indices;
		range.reserve(num_of_agents);
		indices.reserve(num_of_agents);
		int num = 0;
		std::queue<int> Q;
		Q.push(i);
		done[i] = true;
		while (!Q.empty())
		{
			int j = Q.front(); Q.pop();
			range.push_back(0);
			indices.push_back(j);
			for (int k = 0; k < num_of_agents; k++)
			{
				if (WDG[j * num_of_agents + k] > 0)
				{
					range[num] = std::max(range[num], WDG[j * num_of_agents + k]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}		
				else if (WDG[k * num_of_agents + j] > 0)
				{
					range[num] = std::max(range[num], WDG[k * num_of_agents + j]);
					if (!done[k])
					{
						Q.push(k);
						done[k] = true;
					}
				}
			}
			num++;
		}
		if (num  == 1) // no edges
			continue;
		else if (num == 2) // only one edge
		{
			rst += std::max(WDG[indices[0] * num_of_agents + indices[1]], WDG[indices[1] * num_of_agents + indices[0]]); // add edge weight
			continue;
		}
		std::vector<int> G(num * num, 0);
		for (int j = 0; j < num; j++)
		{
			for (int k = j + 1; k < num; k++)
			{
				G[j * num + k] = std::max(WDG[indices[j] * num_of_agents + indices[k]], WDG[indices[k] * num_of_agents + indices[j]]);
			}
		}
		if (num > ILP_node_threshold)
		{
		    rst += greedyWeightedMatching(G, num);
			// rst += ILPForWMVC(G, range);
		}
		else
		{
			std::vector<int> x(num);
			int best_so_far = MAX_COST;
			rst += DPForWMVC(x, 0, 0, G, range, best_so_far);
		}
		double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
		if (runtime > time_limit)
			return -1; // run out of time
	}

	return rst;
}

int CBSDTHeuristic::greedyWeightedMatching(const std::vector<int>& WDG,  int cols)
{
    int rst = 0;
    std::vector<bool> used(cols, false);
    while(true)
    {
        int maxWeight = 0;
        int ep1, ep2;
        for (int i = 0; i < cols; i++)
        {
            if(used[i])
                continue;
            for (int j = i + 1; j < cols; j++)
            {
                if (used[j])
                    continue;
                else if (maxWeight < WDG[i * cols + j])
                {
                    maxWeight = WDG[i * cols + j];
                    ep1 = i;
                    ep2 = j;
                }
            }
        }
        if (maxWeight == 0)
            return rst;
        rst += maxWeight;
        used[ep1] = true;
        used[ep2] = true;
    }
}

// recusive component of weighted vertex cover
int CBSDTHeuristic::DPForWMVC(std::vector<int>& x, int i, int sum, const std::vector<int>& WDG,
	const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return MAX_COST;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForWMVC(x, i + 1, sum, WDG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}
	
	int cols = x.size();
	
	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] < WDG[j * cols + i]) // infeasible assignment
		{
			min_cost = WDG[j * cols + i] - x[j]; // cost should be at least CG[i][j] - x[j];
		}
	}


	int best_cost = -1;
	for (int cost = min_cost; cost <= range[i]; cost++)
	{
		x[i] = cost;
		int rst = DPForWMVC(x, i + 1, sum + x[i], WDG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
			best_cost = cost;
		}
	}
	if (best_cost >= 0)
	{
		x[i] = best_cost; 
	}

	return best_so_far;
}

// recusive component of weighted vertex cover
int CBSDTHeuristic::DPForConstrainedWMVC(std::vector<bool>& x, int i, int sum, const std::vector<int>& WDG, const std::vector<int>& range, int& best_so_far)
{
	if (sum >= best_so_far)
		return INT_MAX;
	double runtime = (double)(clock() - start_time) / CLOCKS_PER_SEC;
	if (runtime > time_limit)
		return -1; // run out of time
	else if (i == (int)x.size())
	{
		best_so_far = sum;
		return sum;
	}
	else if (range[i] == 0) // vertex i does not have any edges.
	{
		int rst = DPForConstrainedWMVC(x, i + 1, sum, WDG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
		return best_so_far;
	}

	int cols = x.size();

	// find minimum cost for this vertex
	int min_cost = 0;
	for (int j = 0; j < i; j++)
	{
		if (min_cost + x[j] * range[j] < WDG[j * cols + i]) // infeasible assignment
		{
			min_cost = WDG[j * cols + i] - x[j] * range[j]; // cost should be at least CG[i][j] - x[j];
		}
	}
	if (min_cost == 0)
	{
		x[i] = 0;
		int rst = DPForConstrainedWMVC(x, i + 1, sum, WDG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
	}
	if (min_cost < range[i])
	{
		x[i] = 1;
		int rst = DPForConstrainedWMVC(x, i + 1, sum + x[i] * range[i], WDG, range, best_so_far);
		if (rst < best_so_far)
		{
			best_so_far = rst;
		}
	}
	return best_so_far;
}

