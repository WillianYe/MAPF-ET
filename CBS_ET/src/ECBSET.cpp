#include "ECBSET.h"
#include <cstdint>
#include "common.h"

// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
// also, do the same for ll_min_f_vals and paths_costs (since its already "on the way").
void ECBSET::updatePaths(ECBSNodeDT* curr)
{
	for (int i = 0; i < num_of_agents; i++)
	{
		paths_consistent[i] = &paths_found_initially[i].first;
		min_travel_times[i] = paths_found_initially[i].second;   
		min_f_vals[i] = abs(min_travel_times[i] - due_times[i]);
        //todo:加入系数
	}
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto & path_pair : curr->paths)
		{
			int agent = path_pair.first;
			if (!updated[agent])
			{
				paths_consistent[agent] = &path_pair.second.first;
				min_travel_times[agent] = path_pair.second.second;
                min_f_vals[agent] = abs(min_travel_times[agent] - due_times[agent]);
				updated[agent] = true;
			}
		}
		curr = curr->parent;
	}

	// 新增
	for (int i = 0; i < num_of_agents; i++)
	{
		laxity_times[i] = min_f_vals[i] < due_times[i]? due_times[i]-min_f_vals[i]:0;
	}
}

bool ECBSET::findPathForSingleAgent(ECBSNodeDT*  node, int ag)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);
	// find a path
	auto new_path = search_engines_dt[ag]->findSuboptimalPath(*node, initial_constraints[ag], paths_consistent, ag, min_f_vals[ag], suboptimality);
	num_LL_expanded += search_engines_dt[ag]->num_expanded;
	num_LL_generated += search_engines_dt[ag]->num_generated;
	runtime_build_CT += search_engines_dt[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines_dt[ag]->runtime_build_CAT;
	runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
	if (new_path.first.empty())
		return false;
	assert(!isSamePath(*paths_consistent[ag], new_path.first));
	node->paths.emplace_back(ag, new_path); 
    //todo:加入系数
    int penalty = abs((int)new_path.first.size() -1 - due_times[ag]);
    int g_penalty = abs((int)new_path.second - due_times[ag]);
    if (obj==0)//max ET
    {
        node->collective_obj = max(node->collective_obj, penalty);	
        node->g_val = max(node->g_val, g_penalty);	
        min_f_vals[ag] = g_penalty;		
    }
    else if (obj==1)//total ET
    {
        int old_panelty = abs((int)paths_consistent[ag]->size() -1 - due_times[ag]);
        node->collective_obj = node->collective_obj - old_panelty + penalty;
        node->g_val = node->g_val - min_f_vals[ag] + g_penalty;
        min_f_vals[ag] = g_penalty;
    }
	
	min_travel_times[ag] = (int)new_path.second;
	node->sum_of_costs = node->sum_of_costs - (int) paths_consistent[ag]->size() + (int) new_path.first.size();
	paths_consistent[ag] = &node->paths.back().second.first;	
	laxity_times[ag] = min_f_vals[ag] < due_times[ag]? due_times[ag]- min_f_vals[ag]: 0;
	node->makespan = max(node->makespan, new_path.first.size() - 1);
	return true;
}

bool ECBSET::generateChild(ECBSNodeDT*  node, ECBSNodeDT* parent)
{
	clock_t t1 = clock();
	node->parent = parent;
	node->HLNode::parent = parent;
	node->g_val = parent->g_val;
	node->collective_obj = parent->collective_obj;
	node->sum_of_costs = parent->sum_of_costs;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());
	for (auto agent : agents)
	{
		if (!findPathForSingleAgent(node, agent))
		{
            if (screen > 1)
                cout << "	No paths for agent " << agent << ". Node pruned." << endl;
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	findConflicts(*node);
	heuristic_helper_dt.computeQuickHeuristics(*node);
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

void ECBSET::computeConflictPriority(shared_ptr<Conflict>& con, ECBSNodeDT& node)
{
    int a1 = con->a1, a2 = con->a2;
	int timestep = get<3>(con->constraint1.back());
	constraint_type type = get<4>(con->constraint1.back());
	bool cardinal1 = false, cardinal2 = false;
	MDD *mdd1 = nullptr, *mdd2 = nullptr;
	if (timestep >= (int)paths_consistent[a1]->size())
		cardinal1 = true;
	else //if (!paths[a1]->at(0).is_single())
	{
		mdd1 = mdd_helper.getMDD(node, a1);
	}
	if (timestep >= (int)paths_consistent[a2]->size())
		cardinal2 = true;
	else //if (!paths[a2]->at(0).is_single())
	{
		mdd2 = mdd_helper.getMDD(node, a2);
	}

	if (type == constraint_type::EDGE) // Edge conflict
	{
		if (timestep < (int)mdd1->levels.size())
		{
			cardinal1 = mdd1->levels[timestep].size() == 1 &&
				mdd1->levels[timestep].front()->location == paths_consistent[a1]->at(timestep) &&
				mdd1->levels[timestep - 1].size() == 1 &&
				mdd1->levels[timestep - 1].front()->location == paths_consistent[a1]->at(timestep - 1);
		}
		if (timestep < (int)mdd2->levels.size())
		{
			cardinal2 = mdd2->levels[timestep].size() == 1 &&
				mdd2->levels[timestep].front()->location == paths_consistent[a2]->at(timestep) &&
				mdd2->levels[timestep - 1].size() == 1 &&
				mdd2->levels[timestep - 1].front()->location == paths_consistent[a2]->at(timestep - 1);
		}
	}
	else // vertex conflict or target conflict
	{
		if (!cardinal1 && timestep < (int)mdd1->levels.size())
		{
			cardinal1 = mdd1->levels[timestep].size() == 1 &&
				mdd1->levels[timestep].front()->location == paths_consistent[a1]->at(timestep);
		}
		if (!cardinal2 && timestep < (int)mdd2->levels.size())
		{
			cardinal2 = mdd2->levels[timestep].size() == 1 &&
				mdd2->levels[timestep].front()->location == paths_consistent[a2]->at(timestep);
		}
	}

	if (cardinal1 && cardinal2)
	{
		con->priority = conflict_priority::CARDINAL;
	}
	else if (cardinal1 || cardinal2)
	{
		con->priority = conflict_priority::SEMI;
	}
	else
	{
		con->priority = conflict_priority::NON;
	}
}

void ECBSET::classifyConflicts(ECBSNodeDT &node)
{
    if (node.unknownConf.empty())
        return;
	// Classify all conflicts in unknownConf
	while (!node.unknownConf.empty())
	{
		shared_ptr<Conflict> con = node.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int timestep = get<3>(con->constraint1.back());
		constraint_type type = get<4>(con->constraint1.back());
		node.unknownConf.pop_front();

		if (PC)
		    if (node.chosen_from == "cleanup" ||
               // (min_f_vals[a1] * suboptimality >= min_f_vals[a1] + 1 &&
               //min_f_vals[a2] * suboptimality >= min_f_vals[a2] + 1))
               (int)paths_consistent[a1]->size() - 1 == min_f_vals[a1] ||
               (int)paths_consistent[a2]->size() - 1 == min_f_vals[a2]) // the path of at least one agent is its shortest path
			    computeConflictPriority(con, node);

		/*if (con->priority == conflict_priority::CARDINAL && heuristic_helper_dt.type == heuristics_type::ZERO)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			return;
		}*/

		// TODO: Mutex reasoning for ECBS
		/*if (mutex_reasoning)
		{
			// TODO mutex reasoning is per agent pair, don't do duplicated work...
			auto mdd1 = mdd_helper.getMDD(node, a1, paths[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths[a2]->size());

			auto mutex_conflict = mutex_helper.run(a1, a2, node, mdd1, mdd2);

			if (mutex_conflict != nullptr)
			{
				computeSecondPriorityForConflict(*mutex_conflict, node);
				node.conflicts.push_back(mutex_conflict);
				continue;
			}
		}*/

		// Target Reasoning
		if (con->type == conflict_type::TARGET)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		if (corridor_reasoning)
		{
			auto corridor = corridor_helper.run(con, paths_consistent, node);
			if (corridor != nullptr)
			{
				corridor->priority = con->priority;
				computeSecondPriorityForConflict(*corridor, node);
				node.conflicts.push_back(corridor);
				continue;
			}
		}

		// Rectangle reasoning
		if (rectangle_reasoning &&
		    (int)paths_consistent[a1]->size() - 1 == min_travel_times[a1] && // the paths for both agents are their shortest paths
		    (int)paths_consistent[a2]->size() - 1 == min_travel_times[a2] &&
            min_travel_times[a1] > timestep &&  //conflict happens before both agents reach their goal locations
			min_travel_times[a2] > timestep &&
			type == constraint_type::VERTEX) // vertex conflict
		{
			auto mdd1 = mdd_helper.getMDD(node, a1, paths_consistent[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths_consistent[a2]->size());
			auto rectangle = rectangle_helper.run(paths_consistent, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
                if (!PC)
                    rectangle->priority = conflict_priority::UNKNOWN;
				computeSecondPriorityForConflict(*rectangle, node);
				node.conflicts.push_back(rectangle);
				continue;
			}
		}

		computeSecondPriorityForConflict(*con, node);
		node.conflicts.push_back(con);
	}

	// remove conflicts that cannot be chosen, to save some memory
	removeLowPriorityConflicts(node.conflicts);
}

inline void ECBSET::pushNode(ECBSNodeDT* node)
{
	num_HL_generated++;
	node->time_generated = num_HL_generated;
	// update handles
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
	case high_level_solver_type::ASTAREPS:  // cleanup_list is called open_list in ECBS
        if (node->collective_obj <= suboptimality * obj_lowerbound)
            node->focal_handle = focal_list.push(node);
            break;
	case high_level_solver_type::NEW:
		if (node->getFHatVal() <= suboptimality * obj_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	case high_level_solver_type::EES:
		node->open_handle = open_list.push(node);
		if (node->getFHatVal() <= suboptimality * inadmissible_obj_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	default:
		break;
	}
	allNodes_table.push_back(node);
}

//只需要考虑node是从cleanup_list中选中的，并且未计算其heuristic的情况
inline bool ECBSET::reinsertNode(ECBSNodeDT* node)
{

	switch (solver_type)
	{
	case high_level_solver_type::ASTAREPS:
        if (node->collective_obj <= suboptimality * obj_lowerbound)
            return false;
        node->cleanup_handle = cleanup_list.push(node);
        break;
	case high_level_solver_type::NEW:
        if (node->getFHatVal() <= suboptimality * obj_lowerbound)
            return false;
        node->cleanup_handle = cleanup_list.push(node);
		break;
	case high_level_solver_type::EES:
        node->cleanup_handle = cleanup_list.push(node);
		node->open_handle = open_list.push(node);
		if (node->getFHatVal() <= suboptimality * inadmissible_obj_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	default:
		break;
	}
	if (screen == 2)
	{
		cout << "	Reinsert " << *node << endl;
	}
	return true;
}

ECBSNodeDT* ECBSET::selectNode()
{
	ECBSNodeDT* curr = nullptr;
	assert(solver_type != high_level_solver_type::ASTAR);
	switch (solver_type)
	{
	case high_level_solver_type::EES:
		// update the focal list if necessary
		// 首先更新focal list，根据open list的top节点，更新focal list
		//此处用不等于进行判断，因为inadmissible，所以新的node和旧的node之间的代价不一定可以比较大小
		if (open_list.top()->getFHatVal() != inadmissible_obj_lowerbound)
		{
			inadmissible_obj_lowerbound = open_list.top()->getFHatVal();
			double focal_list_threshold = suboptimality * inadmissible_obj_lowerbound;
			focal_list.clear();
			for (auto n : open_list)
			{
				if (n->getFHatVal() <= focal_list_threshold)
					n->focal_handle = focal_list.push(n);
			}
		}

		// choose the best node
		// 然后更新下界
		if (screen > 1 && cleanup_list.top()->getFVal() > obj_lowerbound)
			cout << "Lowerbound increases from " << obj_lowerbound << " to " << cleanup_list.top()->getFVal() << endl;
		obj_lowerbound = max(cleanup_list.top()->getFVal(), obj_lowerbound);

		if (focal_list.top()->collective_obj <= suboptimality * obj_lowerbound)
		{ // return best d
			curr = focal_list.top();
			curr->chosen_from = "focal";
			/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
			curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
			curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
			curr->f_of_best_in_open = open_list.top()->getFVal();
			curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
			curr->d_of_best_in_open = open_list.top()->distance_to_go;
			curr->f_of_best_in_focal = focal_list.top()->getFVal();
			curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
			curr->d_of_best_in_focal = focal_list.top()->distance_to_go; */
			focal_list.pop();
			cleanup_list.erase(curr->cleanup_handle);
			open_list.erase(curr->open_handle);
		}
		else if (open_list.top()->collective_obj <= suboptimality * obj_lowerbound)
		{ // return best f_hat
			curr = open_list.top();
			curr->chosen_from = "open";
			/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
			curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
			curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
			curr->f_of_best_in_open = open_list.top()->getFVal();
			curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
			curr->d_of_best_in_open = open_list.top()->distance_to_go;
			curr->f_of_best_in_focal = focal_list.top()->getFVal();
			curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
			curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
			open_list.pop();
			cleanup_list.erase(curr->cleanup_handle);
			focal_list.erase(curr->focal_handle);
		}
		else
		{ // return best f
			curr = cleanup_list.top();
			curr->chosen_from = "cleanup";
			/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
			curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
			curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
			curr->f_of_best_in_open = open_list.top()->getFVal();
			curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
			curr->d_of_best_in_open = open_list.top()->distance_to_go;
			curr->f_of_best_in_focal = focal_list.top()->getFVal();
			curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
			curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
			cleanup_list.pop();
			open_list.erase(curr->open_handle);
			if (curr->getFHatVal() <= suboptimality * inadmissible_obj_lowerbound)
				focal_list.erase(curr->focal_handle);
		}
		break;
	case high_level_solver_type::ASTAREPS:
		// update the focal list if necessary
		//首先判断是否需要更新focal list
		if (cleanup_list.top()->getFVal() > obj_lowerbound)
		{
			if (screen == 3)
			{
				cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << cleanup_list.size() << " to |FOCAL|=";
			}
			double old_focal_list_threshold = suboptimality * obj_lowerbound;
			obj_lowerbound = max(obj_lowerbound, cleanup_list.top()->getFVal());
			double new_focal_list_threshold = suboptimality * obj_lowerbound;
			for (auto n : cleanup_list)
			{
				if (n->collective_obj > old_focal_list_threshold && n->collective_obj <= new_focal_list_threshold)
					n->focal_handle = focal_list.push(n);
			}
			if (screen == 3)
			{
				cout << focal_list.size() << endl;
			}
		}

		// choose best d in the focal list
		curr = focal_list.top();
		curr->chosen_from = "focal";
		/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
		curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
		curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
		curr->f_of_best_in_focal = focal_list.top()->getFVal();
		curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
		curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
		focal_list.pop();
		cleanup_list.erase(curr->cleanup_handle);
		break;
	case high_level_solver_type::NEW:
		// update the focal list if necessary
		if (cleanup_list.top()->getFVal() > obj_lowerbound)
		{
			if (screen == 3)
			{
				cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << cleanup_list.size() << " to |FOCAL|=";
			}
			double old_focal_list_threshold = suboptimality * obj_lowerbound;
			obj_lowerbound = max(obj_lowerbound, cleanup_list.top()->getFVal());
			double new_focal_list_threshold = suboptimality * obj_lowerbound;
			focal_list.clear();
			for (auto n : cleanup_list)
			{
				heuristic_helper_dt.updateInadmissibleHeuristics(*n);
				if (n->getFHatVal() <= new_focal_list_threshold)
					n->focal_handle = focal_list.push(n);
			}
			if (screen == 3)
			{
				cout << focal_list.size() << endl;
			}
		}

		if (focal_list.empty()) // choose best f in the cleanup list (to improve the lower bound)
		{
			curr = cleanup_list.top();
			curr->chosen_from = "cleanup";
			/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
			curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
			curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;*/
			cleanup_list.pop();
		}
		else // choose best d in the focal list
		{
			curr = focal_list.top();
			curr->chosen_from = "focal";
			/*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
			curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
			curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
			curr->f_of_best_in_focal = focal_list.top()->getFVal();
			curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
			curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
			focal_list.pop();
			cleanup_list.erase(curr->cleanup_handle);
		}
		break;
	default:
		break;
	}

	// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
	updatePaths(curr);

	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}

void ECBSET::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << "solution_obj="<< solution_obj << ",solution_cost="<< solution_cost << ",runtime=" << runtime << ",num_HL_expanded=" <<
		num_HL_expanded << ",num_LL_expanded=" << num_LL_expanded << ",obj_lowerbound=" << // HL_num_generated << "," << LL_num_generated << "," <<
		obj_lowerbound << "," << dummy_start->g_val << "," << dummy_start->g_val + dummy_start->h_val << "," <<
		endl;
    /*if (solution_cost >= 0) // solved
    {
        cout << "fhat = [";
        auto curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "hhat = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "d = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->distance_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
        cout << "soc = [";
        curr = goal_node;
        while (curr != nullptr)
        {
            cout << curr->getFHatVal() - curr->cost_to_go << ",";
            curr = curr->parent;
        }
        cout << "]" << endl;
    }*/
}

void ECBSET::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		// addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
		// 	"solution cost,solution obj,min f value,root g value, root f value," <<
		// 	"#adopt bypasses," <<
		// 	"cardinal conflicts," <<
		// 	"standard conflicts,rectangle conflicts,corridor conflicts,target conflicts,mutex conflicts," <<
		// 	"chosen from cleanup,chosen from open,chosen from focal," <<
		// 	"#solve MVCs,#merge MDDs,#solve 2 agents,#memoization," <<
		// 	"cost error,distance error," <<
		// 	"runtime of building heuristic graph,runtime of solving MVC," <<
		// 	"runtime of detecting conflicts," <<
		// 	"runtime of rectangle conflicts,runtime of corridor conflicts,runtime of mutex conflicts," <<
		// 	"runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
		// 	"runtime of path finding,runtime of generating child nodes," <<
		// 	"preprocessing runtime,solver name,instance name" << endl;
		addHeads << "solver name,instance name,agent number,runtime," <<
			"solution cost,solution obj" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	// stats << runtime << "," << 
	// 	num_HL_expanded << "," << num_HL_generated << "," <<
	// 	num_LL_expanded << "," << num_LL_generated << "," <<

	// 	solution_cost << "," <<solution_obj<< "," << obj_lowerbound << "," << dummy_start->g_val << "," <<
	// 	dummy_start->g_val + dummy_start->h_val << "," <<

	// 	num_adopt_bypass << "," <<
	// 	num_cardinal_conflicts << "," <<
	// 	num_standard_conflicts << "," << num_rectangle_conflicts << "," << num_corridor_conflicts << "," << num_target_conflicts << "," << num_mutex_conflicts << "," <<

	// 	num_cleanup << "," << num_open << "," << num_focal << "," <<

	// 	heuristic_helper_dt.num_solve_MVC << "," <<
	// 	heuristic_helper_dt.num_merge_MDDs << "," << 
	// 	heuristic_helper_dt.num_solve_2agent_problems << "," << 
	// 	heuristic_helper_dt.num_memoization << "," <<
	// 	heuristic_helper_dt.getCostError() << "," << heuristic_helper_dt.getDistanceError() << "," <<
	// 	heuristic_helper_dt.runtime_build_dependency_graph << "," << 
	// 	heuristic_helper_dt.runtime_solve_MVC << "," <<

	// 	runtime_detect_conflicts << "," << 
	// 	rectangle_helper.accumulated_runtime << "," << corridor_helper.accumulated_runtime << "," << mutex_helper.accumulated_runtime << "," <<
	// 	mdd_helper.accumulated_runtime << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
	// 	runtime_path_finding << "," << runtime_generate_child << "," <<

	// 	runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
    size_t lastSlashPos = instanceName.rfind('/');
    assert((lastSlashPos != std::string::npos));
    string instanceNameNoPath = instanceName.substr(lastSlashPos + 1);	
	stats << getSolverName() << "," << instanceNameNoPath << "," << num_of_agents << ","
    << runtime << "," << solution_cost << "," << solution_obj << endl;
	stats.close();
}

bool ECBSET::solve(double time_limit, int _obj_lowerbound)
{
	this->obj_lowerbound = _obj_lowerbound;
	this->inadmissible_obj_lowerbound = 0;
	this->time_limit = time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
	// set timer
	start = clock();

	generateRoot();

	//循环A
	while (!cleanup_list.empty() && !solution_found)
	{
		auto curr = selectNode();

		if (terminate(curr))
			return solution_found;

		if ((curr == dummy_start || curr->chosen_from == "cleanup") && !curr->h_computed) 
		// heuristics has not been computed yet
		// runtime reduction techniques: Lazy computation of heuristics. pop and reinsert 
		{
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            bool succ = heuristic_helper_dt.computeInformedHeuristics(*curr, min_f_vals, laxity_times, time_limit - runtime);
            runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            if (!succ) // no solution, so prune this node
            {
                if (screen > 1)
                    cout << "	Prune " << *curr << endl;
                curr->clear();
                continue;
            }

			// 把原来粗略估计的heuristic替换为更细致的heuristic,并重新压入cleanup_list中
            if (reinsertNode(curr))
                continue;
		}

        classifyConflicts(*curr);

		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
		if (bypass && curr->chosen_from != "cleanup")
		{
			bool foundBypass = true;

			//循环B
			while (foundBypass)
			{
				if (terminate(curr))
					return solution_found;
				foundBypass = false;
				ECBSNodeDT* child[2] = { new ECBSNodeDT() , new ECBSNodeDT() };
				curr->conflict = chooseConflict(*curr);
				addConstraints(curr, child[0], child[1]);
				if (screen > 1)
					cout << "	Expand " << *curr << endl << 	"	on " << *(curr->conflict) << endl;

				bool solved[2] = { false, false };
				vector<vector<int>*> path_copy(paths_consistent);
				vector<int> fmin_copy(min_f_vals);
				vector<int> fmin_tt_copy(min_travel_times);

				////循环C
				for (int i = 0; i < 2; i++)
				{
					if (i > 0)
					{
						paths_consistent = path_copy;
						min_f_vals = fmin_copy;
						min_travel_times = fmin_tt_copy;
					}
					solved[i] = generateChild(child[i], curr);
					
					if (!solved[i])
					{
						delete (child[i]);
						continue;
					}
					else if (i == 1 && !solved[0])
						continue;
					else if (bypass &&
						child[i]->collective_obj <= suboptimality * obj_lowerbound &&
						child[i]->distance_to_go < curr->distance_to_go) // Bypass1
					{
						foundBypass = true;
						for (const auto& path : child[i]->paths)
						{
						    /*if (path.second.first.size() != path_copy[path.first]->size()) // CBS bypassing
                            {
                                foundBypass = false;
                                break;
                            }*/
							int temp=abs((int)path.second.first.size() - 1-due_times[path.first]);
							if (temp > suboptimality * fmin_copy[path.first]) // EECBS bypassing
							{
								foundBypass = false;
								break;
							}
						}
						if (foundBypass)
						{
							adoptBypass(curr, child[i], fmin_copy);
							if (screen > 1)
								cout << "	Update " << *curr << endl;
							break;
						}
					}
				} //循环C结束

				if (foundBypass)
				{
					for (auto & i : child)
					{
						delete i;
					}
                    classifyConflicts(*curr); // classify the new-detected conflicts
				}
				else // 没有找到bypass，正常拓展
				{
					for (int i = 0; i < 2; i++)
					{
						if (solved[i])
						{
							pushNode(child[i]);
							curr->children.push_back(child[i]);
							if (screen > 1)
							{
								cout << "		Generate " << *child[i] << endl;
							}
						}
					}
				}
			} ////循环B结束
		}
		else // no bypass
		{
			ECBSNodeDT* child[2] = { new ECBSNodeDT() , new ECBSNodeDT() };
			curr->conflict = chooseConflict(*curr);
			addConstraints(curr, child[0], child[1]);

			if (screen > 1)
				cout << "	Expand " << *curr << endl << "	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<int>*> path_copy(paths_consistent);
			vector<int> fmin_copy(min_f_vals);
			vector<int> fmin_tt_copy(min_travel_times);
			for (int i = 0; i < 2; i++)
			{
				if (i > 0)
				{
					paths_consistent = path_copy;
					min_f_vals = fmin_copy;
					min_travel_times = fmin_tt_copy;
				}
				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					delete (child[i]);
					continue;
				}
				pushNode(child[i]);
				curr->children.push_back(child[i]);
				if (screen > 1)
					cout << "		Generate " << *child[i] << endl;
			}
		}
		switch (curr->conflict->type)
		{
		case conflict_type::RECTANGLE:
			num_rectangle_conflicts++;
			break;
		case conflict_type::CORRIDOR:
			num_corridor_conflicts++;
			break;
		case  conflict_type::TARGET:
			num_target_conflicts++;
			break;
		case conflict_type::STANDARD:
			num_standard_conflicts++;
			break;
		case conflict_type::MUTEX:
			num_mutex_conflicts++;
			break;
		default:
			break;
		}
		if (curr->chosen_from == "cleanup")
			num_cleanup++;
		else if (curr->chosen_from == "open")
			num_open++;
		else if (curr->chosen_from == "focal")
			num_focal++;
		if (curr->conflict->priority == conflict_priority::CARDINAL)
			num_cardinal_conflicts++;
        if (!curr->children.empty())
            heuristic_helper_dt.updateOnlineHeuristicErrors(*curr); // update online heuristic errors
		curr->clear();
	}  // end of while loop

	return solution_found;
}

void ECBSET::adoptBypass(ECBSNodeDT* curr, ECBSNodeDT* child, const vector<int>& fmin_copy)
{
	num_adopt_bypass++;
	curr->sum_of_costs = child->sum_of_costs;
	curr->collective_obj = child->collective_obj;
	curr->cost_to_go = child->cost_to_go;
	curr->distance_to_go = child->distance_to_go;
	curr->conflicts = child->conflicts;
	curr->unknownConf = child->unknownConf;
	curr->conflict = nullptr;
	curr->makespan = child->makespan;
	for (const auto& path_pair : child->paths) // update paths
	{
		auto p = curr->paths.begin();
		while (p != curr->paths.end())
		{
			if (path_pair.first == p->first)
			{
				p->second.first = path_pair.second.first;
				paths_consistent[p->first] = &p->second.first;
				min_travel_times[p->first] = p->second.second;	
                min_f_vals[p->first] = abs(p->second.second - due_times[p->first]);              						
				break;
			}
			++p;
		}
		if (p == curr->paths.end())
		{
			curr->paths.emplace_back(path_pair);
			curr->paths.back().second.second = fmin_copy[path_pair.first];
			paths_consistent[path_pair.first] = &curr->paths.back().second.first;
			min_f_vals[path_pair.first] = fmin_copy[path_pair.first];
		}
	}
}

//判断终止条件，超时、超过上限、找到解等
bool ECBSET::terminate(ECBSNodeDT* curr)
{
	if (obj_lowerbound >= obj_upperbound)
	{
		solution_obj = obj_lowerbound;
		solution_found = false;
        printResults();
		return true;
	}
	runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (curr->conflicts.empty() && curr->unknownConf.empty()) //no conflicts
	{// found a solution
		solution_found = true;
		goal_node = curr;
		if (screen > 0)
			cout<<"final node: soc="<<curr->sum_of_costs<<",obj="<<curr->getFHatVal() - curr->cost_to_go<<endl;
		solution_obj = goal_node->getFHatVal() - goal_node->cost_to_go;
		solution_cost = goal_node->sum_of_costs;
		if (!validateSolution())
		{
			cout << "Solution invalid!!!" << endl;
			printPaths();
			exit(-1);
		}
		printResults();
		return true;
	}
	if (runtime > time_limit || num_HL_expanded > node_limit)
	{   // time/node out
		solution_cost = -1;
		solution_obj = -1;
		solution_found = false;
        printResults();
		return true;
	}
	return false;
}

bool ECBSET::generateRoot()
{
	auto root = new ECBSNodeDT();
    root->g_val = 0;
    root->collective_obj = 0;
	root->sum_of_costs = 0;
	root->makespan = 0;
	paths_consistent.resize(num_of_agents, nullptr);
	min_f_vals.resize(num_of_agents);
	min_travel_times.resize(num_of_agents);
	
	mdd_helper.init(num_of_agents);
	heuristic_helper_dt.init();

	// initialize paths_found_initially
	assert(paths_found_initially.empty());
	paths_found_initially.resize(num_of_agents);
	//generate random permutation of agent indices
	auto agents = shuffleAgents();

	for (auto i : agents)
	{
		paths_found_initially[i] = search_engines_dt[i]->findSuboptimalPath(*root, initial_constraints[i], paths_consistent, i, 0, suboptimality);
		if (paths_found_initially[i].first.empty())
		{
			cerr << "The start-goal locations of agent " << i << "are not connected" << endl;
			exit(-1);
		}
		paths_consistent[i] = &paths_found_initially[i].first;
		min_travel_times[i] = paths_found_initially[i].second;
		root->makespan = max(root->makespan, paths_consistent[i]->size() - 1);
		root->sum_of_costs += (int)paths_consistent[i]->size() - 1;

        int penalty = abs((int)paths_consistent[i]->size() - 1 - due_times[i]);
        min_f_vals[i] = abs(min_travel_times[i]-due_times[i]);
        if (obj==0)//max ET
        {
            root->g_val = max(root->g_val,min_f_vals[i]);
            root->collective_obj = max(root->collective_obj, penalty);	
        }
        else if (obj==1)//total ET
        {       
            root->g_val += min_f_vals[i];
            root->collective_obj += penalty;
        }

		laxity_times[i] = min_f_vals[i] < due_times[i]? due_times[i] - min_f_vals[i]:0;

		num_LL_expanded += search_engines_dt[i]->num_expanded;
		num_LL_generated += search_engines_dt[i]->num_generated;
	}

	root->h_val = 0;
	root->depth = 0;
	findConflicts(*root);
    heuristic_helper_dt.computeQuickHeuristics(*root);
	pushNode(root);
	dummy_start = root;

	return true;
}

inline void ECBSET::releaseNodes()
{
	open_list.clear();
	cleanup_list.clear();
	focal_list.clear();
	for (auto& node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

ECBSET::~ECBSET()
{
	releaseNodes();
	mdd_helper.clear();
}

void ECBSET::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();

	for (auto s : search_engines_dt)
		delete s;
	search_engines_dt.clear();
}

bool ECBSET::validateSolution() const
{
	// check whether the solution cost is within the bound
	if (solution_obj > obj_lowerbound * suboptimality)
    {
	    cout << "Solution cost exceeds the sub-optimality bound!" << endl;
        return false;
    }

	// check whether the paths are feasible
	size_t soc = 0;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		soc += paths_consistent[a1]->size() - 1;
		for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
		{
			size_t min_path_length = paths_consistent[a1]->size() < paths_consistent[a2]->size() ? paths_consistent[a1]->size() : paths_consistent[a2]->size();
			for (size_t timestep = 0; timestep < min_path_length; timestep++)
			{
				int loc1 = paths_consistent[a1]->at(timestep);
				int loc2 = paths_consistent[a2]->at(timestep);
				if (loc1 == loc2)
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
					return false;
				}
				else if (timestep < min_path_length - 1
					&& loc1 == paths_consistent[a2]->at(timestep + 1)
					&& loc2 == paths_consistent[a1]->at(timestep + 1))
				{
					cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
						loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
					return false;
				}
			}
			if (paths_consistent[a1]->size() != paths_consistent[a2]->size())
			{
				int a1_ = paths_consistent[a1]->size() < paths_consistent[a2]->size() ? a1 : a2;
				int a2_ = paths_consistent[a1]->size() < paths_consistent[a2]->size() ? a2 : a1;
				int loc1 = paths_consistent[a1_]->back();
				for (size_t timestep = min_path_length; timestep < paths_consistent[a2_]->size(); timestep++)
				{
					int loc2 = paths_consistent[a2_]->at(timestep);
					if (loc1 == loc2)
					{
						cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
						return false; // It's at least a semi conflict			
					}
				}
			}
		}
	}
	if ((int)soc != solution_cost)
	{
		cout << "The solution cost is wrong!" <<soc<<","<<solution_cost<< endl;
		return false;
	}
	return true;
}

inline int ECBSET::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths_consistent[agent_id]->size() - 1), (size_t)0);
	return paths_consistent[agent_id]->at(t);
}

// used for rapid random  restart
void ECBSET::clear()
{
    mdd_helper.clear();
    heuristic_helper_dt.clear();
	releaseNodes();
    paths_consistent.clear();
    paths_found_initially.clear();
    dummy_start = nullptr;
    goal_node = nullptr;
    solution_found = false;
    solution_cost = -2;
	solution_obj = -2;
}

void ECBSET::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (" << paths_found_initially[i].first.size() - 1 << " -->" <<
			paths_consistent[i]->size() - 1 << "): ";
		for (const auto & t : *paths_consistent[i])
			cout << t << "->";
		cout << endl;
	}
}

string ECBSET::getSolverName() const
{
	string name;
	name += "ECBS-ET ";
	if (disjoint_splitting)
		name += "Disjoint ";
	switch (heuristic_helper_dt.type)
	{
	case heuristics_type::ZERO:
		if (PC)
			name += "ICBS";
		else
			name += "CBS";
		break;
	case heuristics_type::WDG:
		name += "WDG";
		break;
	default:
		break;
	}
	if (rectangle_reasoning)
		name += "+R";
	if (corridor_reasoning)
		name += "+C";
	if (target_reasoning)
		name += "+T";
	if (mutex_reasoning)
		name += "+MP";
	if (bypass)
		name += "+BP";
	// name += " with " + search_engines_dt[0]->getName();
    name += " with obj " + std::to_string(obj);
	return name;
}