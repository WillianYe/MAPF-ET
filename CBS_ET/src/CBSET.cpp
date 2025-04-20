#include <algorithm>    // std::shuffle
#include <cstdint>
#include <random>      // std::default_random_engine
#include "CBSET.h"
#include "CBSNodeDT.h"
#include "common.h"


//从当前HLNode开始回溯到根节点，直到建立与当前HLNode一致的全部agent集合的consistent路径
// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBSET::updatePaths(CBSNodeDT* curr)
{
	for (int i = 0; i < num_of_agents; i++)
		paths_consistent[i] = &paths_found_initially[i];
	//每个agent只需要留下最新的路径。因此需要一个flag记录该agent是否有路径。记录每个agent的最后一次更新
	vector<bool> updated(num_of_agents, false);  // initialized for false

	while (curr != nullptr)
	{
		for (auto & path_pair : curr->paths)
		{
			if (!updated[path_pair.first])
			{
				paths_consistent[path_pair.first] = &(path_pair.second);
				updated[path_pair.first] = true;
			}
		}
		curr = curr->parent;
	}

	// 新增
	for (int i = 0; i < num_of_agents; i++)
	{
		laxity_times[i] = paths_consistent[i]->size() -1 < due_times[i]? due_times[i]-paths_consistent[i]->size() + 1:0;
	}
}

//检测冲突、选择冲突、生成约束之后，为约束中的新agent生成与当前节点约束一致的路径
bool CBSET::findPathForSingleAgent(CBSNodeDT*  node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);
	// find a path
	Path new_path = search_engines_dt[ag]->findOptimalPath(*node, initial_constraints[ag], paths_consistent, ag, lowerbound);
	num_LL_expanded += search_engines_dt[ag]->num_expanded;
	num_LL_generated += search_engines_dt[ag]->num_generated;
	runtime_build_CT += search_engines_dt[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines_dt[ag]->runtime_build_CAT;
	runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
	if (!new_path.empty())
	{
		assert(!isSamePath(*paths_consistent[ag], new_path));
		node->paths.emplace_back(ag, new_path);
		int penalty = abs((int)new_path.size() -1 - due_times[ag]);
        //todo:加入系数
		if (obj==0)//max ET
		{
            node->g_val = max(node->g_val, penalty);			
		}
		else if (obj==1)//total ET
		{
			int old_panelty = abs((int)paths_consistent[ag]->size()  -1 - due_times[ag]);
			node->g_val = node->g_val - old_panelty + penalty;
		}

		node->sum_of_costs = node->sum_of_costs - (int)paths_consistent[ag]->size() + (int)new_path.size();		
		paths_consistent[ag] = &node->paths.back().second;
		laxity_times[ag] = (int)paths_consistent[ag]->size() -1 < due_times[ag]? due_times[ag]-(int)paths_consistent[ag]->size() +1: 0;
		node->makespan = max(node->makespan, new_path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}

//把node作为parent的子节点，并检测、更新其冲突
bool CBSET::generateChild(CBSNodeDT*  node, CBSNodeDT* parent)
{
	clock_t t1 = clock();
	//区分不同的parent
	node->parent = parent;
	node->HLNode::parent = parent; 
	node->g_val = parent->g_val;
	node->sum_of_costs = parent->sum_of_costs;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;
	
	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());
	for (auto agent : agents)
	{
		int lowerbound = (int)paths_consistent[agent]->size() - 1;
		//新路径必定必定不小于lowerbound
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	findConflicts(*node);
	heuristic_helper_dt.computeQuickHeuristics(*node);
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

// 方法：通过MDD
// 供classifyConflicts函数调用
//输出：CARDINAL, PSEUDO_CARDINAL, SEMI, NON
void CBSET::computeConflictPriority(shared_ptr<Conflict>& con, CBSNodeDT& node)
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
		mdd1 = mdd_helper.getMDD(node, a1, paths_consistent[a1]->size());
	}

	if (timestep >= (int)paths_consistent[a2]->size())
		cardinal2 = true;
	else //if (!paths[a2]->at(0).is_single())
	{
		mdd2 = mdd_helper.getMDD(node, a2, paths_consistent[a2]->size());
	}

	if (type == constraint_type::EDGE) // Edge conflict
	{
		cardinal1 = mdd1->levels[timestep].size() == 1 && mdd1->levels[timestep - 1].size() == 1;
		cardinal2 = mdd2->levels[timestep].size() == 1 && mdd2->levels[timestep - 1].size() == 1;
	}
	else // vertex conflict or target conflict
	{
		if (!cardinal1)
			cardinal1 = mdd1->levels[timestep].size() == 1;
		if (!cardinal2)
			cardinal2 = mdd2->levels[timestep].size() == 1;
	}

	/*int width_1 = 1, width_2 = 1;

	if (paths[a1]->size() > timestep){
	width_1 = paths[a1]->at(timestep).mdd_width;
	}

	if (paths[a2]->size() > timestep){
	width_2 = paths[a2]->at(timestep).mdd_width;
	}
	con -> mdd_width = width_1 * width_2;*/

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

// findConflicts 初步找到了STANDARD（点、边）和TARGET冲突并放到unknownConf中。
// 本函数继续进行推理，找到更多冲突类型，并放到conflicts中
void CBSET::classifyConflicts(CBSNodeDT &node)
{
	// Classify all conflicts in unknownConf
	// unknownConf store all conflicts
	while (!node.unknownConf.empty())
	{
		shared_ptr<Conflict> con = node.unknownConf.front();
		int a1 = con->a1, a2 = con->a2;
		int timestep = get<3>(con->constraint1.back());
		constraint_type type = get<4>(con->constraint1.back());
		//int a, loc1, loc2, timestep;
		//constraint_type type;
		//tie(a, loc1, loc2, timestep, type) = con->constraint1.back();
		node.unknownConf.pop_front();

		computeConflictPriority(con, node);

		// TODO？CARDINAL的冲突一定是STANDARD（点和边）？？
		if (con->priority == conflict_priority::CARDINAL && heuristic_helper_dt.type == heuristics_type::ZERO)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			return;
		}

		// // Mutex reasoning. TODO
		// if (mutex_reasoning)
		// {
		// 	// TODO mutex reasoning is per agent pair, don't do duplicated work...
		// 	auto mdd1 = mdd_helper.getMDD(node, a1, paths_consistent[a1]->size());
		// 	auto mdd2 = mdd_helper.getMDD(node, a2, paths_consistent[a2]->size());

		// 	auto mutex_conflict = mutex_helper.run(a1, a2, node, mdd1, mdd2);

		// 	if (mutex_conflict != nullptr)
		// 	{
		// 		computeSecondPriorityForConflict(*mutex_conflict, node);
		// 		node.conflicts.push_back(mutex_conflict);
		// 		continue;
		// 	}
		// }

		// Target Reasoning  findConflicts函数可以识别出来target conflict
		if (con->type == conflict_type::TARGET)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			continue;
		}

		// Corridor reasoning
		if (corridor_reasoning)
		{
			// 冲突类型为type = conflict_type::CORRIDOR
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
			(int)paths_consistent[con->a1]->size() > timestep &&
			(int)paths_consistent[con->a2]->size() > timestep && //conflict happens before both agents reach their goal locations
			type == constraint_type::VERTEX) // vertex conflict
		{
			auto mdd1 = mdd_helper.getMDD(node, a1, paths_consistent[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths_consistent[a2]->size());
			// type = conflict_type::RECTANGLE
			auto rectangle = rectangle_helper.run(paths_consistent, timestep, a1, a2, mdd1, mdd2);
			if (rectangle != nullptr)
			{
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

inline void CBSET::pushNode(CBSNodeDT* node)
{
	num_HL_generated++;
	node->time_generated = num_HL_generated;
	// update handles
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
		case high_level_solver_type::ASTAR:
			break;
		case high_level_solver_type::ASTAREPS: // cleanup_list is called open_list in ECBS
			if (node->getFVal()<= suboptimality * obj_lowerbound)
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
	}
	allNodes_table.push_back(node);
}

// runtime reduction techniques： lazy computation
// 将pop出来的节点，重新计算更informed的h值，并重新压入
inline bool CBSET::reinsertNode(CBSNodeDT* node)
{
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
	
	//前面直接将pop的当前node压入cleanup_list，A*啥都不用做
	case high_level_solver_type::ASTAR:
		break;
	
	//A*EPS还要重新压入focal
	case high_level_solver_type::ASTAREPS:
		if (node->getFVal() <= suboptimality * obj_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	case high_level_solver_type::NEW:
        if (node->getFHatVal() <= suboptimality * obj_lowerbound)
            node->focal_handle = focal_list.push(node);
		break;

	//EES还要重新压入open和focal
	case high_level_solver_type::EES:	
		node->open_handle = open_list.push(node);
		if (node->getFHatVal() <= suboptimality * inadmissible_obj_lowerbound)
			node->focal_handle = focal_list.push(node);
		break;
	}
	if (screen == 2)
	{
		cout << "	Reinsert " << *node << endl;
	}
	return true;
}

//根据A×，FS(f=g)，FS(f=g+h)，EES寻找顶部节点
CBSNodeDT* CBSET::selectNode()
{
	CBSNodeDT* curr = nullptr;
	switch (solver_type)
	{
        case high_level_solver_type::ASTAR:
			//提高和更新下界
            obj_lowerbound = max(obj_lowerbound, cleanup_list.top()->getFVal());
            curr = cleanup_list.top();
            curr->chosen_from = "cleanup";
            /*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
            curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
            curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;*/
            cleanup_list.pop();
            break;
        case high_level_solver_type::ASTAREPS:
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
                for (auto n : cleanup_list)
                {
                    if (n->getFVal() > old_focal_list_threshold && n->getFVal() <= new_focal_list_threshold)
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
        case high_level_solver_type::EES:
            // update the focal list if necessary
            if (open_list.top()->getFHatVal() > inadmissible_obj_lowerbound)
            {
                if (screen == 3)
                {
                    cout << "  Note -- FOCAL UPDATE!! from |FOCAL|=" << focal_list.size() << " with |OPEN|=" << open_list.size() << " to |FOCAL|=";
                }
                double old_focal_list_threshold = suboptimality * inadmissible_obj_lowerbound;
                inadmissible_obj_lowerbound = open_list.top()->getFHatVal();
                double new_focal_list_threshold = suboptimality * inadmissible_obj_lowerbound;
                for (auto n : open_list)
                {
					//增量式更新
                    if (n->getFHatVal() > old_focal_list_threshold &&
                        n->getFHatVal() <= new_focal_list_threshold)
                        n->focal_handle = focal_list.push(n);
                }
                if (screen == 3)
                {
                    cout << focal_list.size() << endl;
                }
            }

            // choose the best node
            obj_lowerbound = max(cleanup_list.top()->getFVal(), obj_lowerbound);
            if (focal_list.top()->getFVal() <= suboptimality * obj_lowerbound)
            { // return best d
                curr = focal_list.top();
                /* for debug */
                curr->chosen_from = "focal";
                curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
                curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
                curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
                curr->f_of_best_in_open = open_list.top()->getFVal();
                curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
                curr->d_of_best_in_open = open_list.top()->distance_to_go;
                curr->f_of_best_in_focal = focal_list.top()->getFVal();
                curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
                curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
                /* end for debug */
                focal_list.pop();
                cleanup_list.erase(curr->cleanup_handle);
                open_list.erase(curr->open_handle);
            }
            else if (open_list.top()->getFVal() <= suboptimality * obj_lowerbound)
            { // return best f_hat
                curr = open_list.top();
                /* for debug */
                curr->chosen_from = "open";
                curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
                curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
                curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
                curr->f_of_best_in_open = open_list.top()->getFVal();
                curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
                curr->d_of_best_in_open = open_list.top()->distance_to_go;
                curr->f_of_best_in_focal = focal_list.top()->getFVal();
                curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
                curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
                /* end for debug */
                open_list.pop();
                cleanup_list.erase(curr->cleanup_handle);
                focal_list.erase(curr->focal_handle);
            }
            else
            { // return best f
                curr = cleanup_list.top();
                /* for debug */
                curr->chosen_from = "cleanup";
                curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
                curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
                curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
                curr->f_of_best_in_open = open_list.top()->getFVal();
                curr->f_hat_of_best_in_open = open_list.top()->getFHatVal();
                curr->d_of_best_in_open = open_list.top()->distance_to_go;
                curr->f_of_best_in_focal = focal_list.top()->getFVal();
                curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
                curr->d_of_best_in_focal = focal_list.top()->distance_to_go;
                /* end for debug */
                cleanup_list.pop();
                open_list.erase(curr->open_handle);
                if (curr->getFHatVal() <= suboptimality * inadmissible_obj_lowerbound)
                    focal_list.erase(curr->focal_handle);
            }
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
                for (auto n : cleanup_list)
                {
                    if (n->getFHatVal() > old_focal_list_threshold && n->getFHatVal() <= new_focal_list_threshold)
                        n->focal_handle = focal_list.push(n);
                }
                if (screen == 3)
                {
                    cout << focal_list.size() << endl;
                }
            }
            obj_lowerbound = cleanup_list.top()->getFVal();
            if (focal_list.empty())
            {
                // choose best f in the cleanup list (to improve the lower bound)
                curr = cleanup_list.top();
                curr->chosen_from = "cleanup";
                /*curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
                curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
                curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;*/
                cleanup_list.pop();
            }
            else
            {
                // choose best d in the focal list
                curr = focal_list.top();
                /*curr->chosen_from = "focal";
                curr->f_of_best_in_cleanup = cleanup_list.top()->getFVal();
                curr->f_hat_of_best_in_cleanup = cleanup_list.top()->getFHatVal();
                curr->d_of_best_in_cleanup = cleanup_list.top()->distance_to_go;
                curr->f_of_best_in_focal = focal_list.top()->getFVal();
                curr->f_hat_of_best_in_focal = focal_list.top()->getFHatVal();
                curr->d_of_best_in_focal = focal_list.top()->distance_to_go;*/
                focal_list.pop();
                cleanup_list.erase(curr->cleanup_handle);
            }
            break;
	}

	// takes the paths_found_initially and UPDATE all constrained paths found for agents from curr to dummy_start (and lower-bounds)
	// CBS主流程维持当前节点对应的全体路径
	updatePaths(curr);
	// printPaths();

	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}



void CBSET::printResults() const
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

void CBSET::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		// addHeads << "solver name,instance name,runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
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
		// 	"preprocessing runtime" << endl;
		addHeads << "solver name,instance name,agent number,runtime," <<
			"solution cost,solution obj" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	// stats << getSolverName() << "," << instanceNameNoPath << "," << runtime << "," << 
	// 	solution_cost << "," << solution_obj << "," << obj_lowerbound << "," << dummy_start->g_val << "," <<
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

	// 	runtime_preprocessing  << endl;
    size_t lastSlashPos = instanceName.rfind('/');
    assert((lastSlashPos != std::string::npos));
    string instanceNameNoPath = instanceName.substr(lastSlashPos + 1);	
	stats << getSolverName() << "," << instanceNameNoPath << "," << num_of_agents << ","
    << runtime << "," << solution_cost << "," << solution_obj << endl;
	stats.close();
}


//算法主流程
bool CBSET::solve(double _time_limit, int _obj_lowerbound, int _obj_upperbound)
{
	this->obj_lowerbound = _obj_lowerbound;
	this->inadmissible_obj_lowerbound = 0;
	this->obj_upperbound = _obj_upperbound;
	this->time_limit = _time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": "<<endl;
	}
	// set timer
	start = clock();

	generateRoot();

	while (!cleanup_list.empty() && !solution_found)
	{
		auto curr = selectNode();

		// 会pop node
		if(screen>2){
			cout<<"select node soc="<<curr->sum_of_costs<<", g_val="<<curr->g_val<<endl; 
			printPaths();
		}
			
		if (terminate(curr))
			return solution_found;

		if (PC) // priortize conflicts
			classifyConflicts(*curr);

		// runtime reduction techniques. 采取lazy策略，之前计算的h是粗略的，所以需要重新计算，并重新压入cleanup_list
		if (!curr->h_computed) // heuristics has not been computed yet
		{
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
			bool succ = heuristic_helper_dt.computeInformedHeuristics(*curr, laxity_times, time_limit - runtime);
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
			
			if (!succ) // no solution, so prune this node
			{
				curr->clear();
				continue;
			}

			// 计算完了h，重新压入cleanup_list。如果是其他上层算法，还要判断是否压入focal_list和open_list
			// 压入之后，重新开始while循环
			if (reinsertNode(curr))
			{
				continue;
			}
		}

		//Expand the node
		num_HL_expanded++;
		curr->time_expanded = num_HL_expanded;
		bool foundBypass = true;
		while (foundBypass)
		{
			if(terminate(curr))
				return solution_found;
			foundBypass = false;
			CBSNodeDT* child[2] = { new CBSNodeDT() , new CBSNodeDT() };

			//按照优先级选择约束
			curr->conflict = chooseConflict(*curr);

			addConstraints(curr, child[0], child[1]);

			if (screen >= 1)
				cout << "	Expand " << *curr << endl <<"	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<int>*> copy(paths_consistent);

			for (int i = 0; i < 2; i++)
			{
				//如果左孩子不能bypass，则继续检查右孩子
				// i=0之后，path_consistent可能更新过，所以要为i=1的时候保存paths_consistent的一个拷贝
				if (i > 0)
					paths_consistent = copy;

				solved[i] = generateChild(child[i], curr);
				if (!solved[i])
				{
					delete (child[i]);
					continue;
				}
				else if (bypass && child[i]->g_val == curr->g_val && child[i]->distance_to_go < curr->distance_to_go) // Bypass1
				{
					//若为右孩子，且左孩子无解，则没必要继续检查右孩子了
					if (i == 1 && !solved[0])
						continue;
					foundBypass = true;
					
					//以下为adopt bypass子功能
					num_adopt_bypass++;
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->distance_to_go = child[i]->distance_to_go;
					curr->sum_of_costs = child[i]->sum_of_costs; //重要！g_val相同的话，需要更新sum_of_costs和makespan
					curr->makespan = child[i]->makespan; //重要！
					curr->conflict = nullptr;
					
					for (const auto& path_pair : child[i]->paths) // update paths
					{
						auto p = curr->paths.begin();
						while (p != curr->paths.end())
						{
							if (path_pair.first == p->first)
							{
								p->second = path_pair.second;				
								paths_consistent[p->first] = &p->second;	
								break;
							}
							++p;
						}
						//如果path_pair中出现了新的agent_id，则直接添加
						if (p == curr->paths.end())
						{
							curr->sum_of_costs -= (int)paths_consistent[path_pair.first]->size() - 1;
							curr->paths.emplace_back(path_pair);
							paths_consistent[path_pair.first] = &curr->paths.back().second;
							curr->sum_of_costs += (int)path_pair.second.size() - 1;
							curr->makespan = max((int)curr->makespan, (int)path_pair.second.size() - 1);
						}						
					}
					
					if (screen > 1)
					{
						cout << "	Update " << *curr << endl;		
					}
					break;
				}
			}

			//若找到了bypass，则删除所有子节点，仅保留更新后的curr
			if (foundBypass)
			{				
				for (auto & i : child)
				{
					delete i;
					i = nullptr;
				}
				if (PC) // prioritize conflicts
					classifyConflicts(*curr);
			}
			else //正常添加child[0]和child[1]
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
				curr->clear();
			}
		}
	}  // end of while loop
	return solution_found;
}

//判断终止条件，超时、超过上限、找到解等
bool CBSET::terminate(CBSNodeDT* curr)
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
		// if (!validateSolution())
		// {
		// 	cout << "Solution invalid!!!" << endl;
		// 	printPaths();
		// 	exit(-1);
		// }
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

bool CBSET::generateRoot()
{
	auto root = new CBSNodeDT();
	root->g_val = 0;

	root->sum_of_costs = 0;
	paths_consistent.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper_dt.init();

	// initialize paths_found_initially
	if (paths_found_initially.empty())
	{
		paths_found_initially.resize(num_of_agents);

		//generate random permutation of agent indices
		vector<int> agents(num_of_agents);
		for (int i = 0; i < num_of_agents; i++)
		{
			agents[i] = i;
		}

		if (randomRoot)
		{
			std::random_device rd;//非确定性随机数生成设备。真随机数。rd()为一个结果。结果均匀分布在闭合范围 [0, 2^32) 中
			std::mt19937 g(rd()); //随机数引擎
			std::shuffle(std::begin(agents), std::end(agents), g);
		}

		for (auto i : agents)
		{
			//CAT cat(dummy_start->makespan + 1);  // initialized to false
			//updateReservationTable(cat, i, *dummy_start);
			
			paths_found_initially[i] = search_engines_dt[i]->findOptimalPath(*root, initial_constraints[i], paths_consistent, i, 0);
			if (paths_found_initially[i].empty())
			{
				if (screen >= 2)
					cout << "No path exists for agent " << i << endl;
				delete root;
				return false;
			}
			paths_consistent[i] = &paths_found_initially[i];
			laxity_times[i] = paths_consistent[i]->size() -1 < due_times[i]? due_times[i]-paths_consistent[i]->size() +1: 0;
            
            int penalty = abs((int)paths_found_initially[i].size() -1 - due_times[i]);
			if (obj==0){
                root->g_val = max(root->g_val, penalty);	
            }
			else if (obj==1){
                root->g_val += penalty;
            }

			root->makespan = std::max((int)root->makespan, (int)paths_found_initially[i].size() - 1);
			root->sum_of_costs += (int)paths_found_initially[i].size() - 1;
			num_LL_expanded += search_engines_dt[i]->num_expanded;
			num_LL_generated += search_engines_dt[i]->num_generated;
		}
	}
	else
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			paths_consistent[i] = &paths_found_initially[i];
			laxity_times[i] = (int)paths_consistent[i]->size() -1 < due_times[i]? due_times[i]-(int)paths_consistent[i]->size() +1: 0;
			
            int penalty = abs((int)paths_found_initially[i].size() -1 - due_times[i]);
			if (obj==0){
                root->g_val = max(root->g_val, penalty);	
            }
			else if (obj==1){
                root->g_val += penalty;
            }
			root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
			root->sum_of_costs += (int) paths_found_initially[i].size() - 1;
		}
	}

	root->h_val = 0;
	root->depth = 0;
	findConflicts(*root);
	heuristic_helper_dt.computeQuickHeuristics(*root);
	pushNode(root);
	dummy_start = root;

	return true;
}

inline void CBSET::releaseNodes()
{
	open_list.clear();
	cleanup_list.clear();
	focal_list.clear();
	for (auto& node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

CBSET::~CBSET()
{
	releaseNodes();
	mdd_helper.clear();
}

void CBSET::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();

	for (auto s : search_engines_dt)
		delete s;
	search_engines_dt.clear();
}

bool CBSET::validateSolution() const
{
	// check whether the solution cost is within the bound
	if (solution_obj > obj_lowerbound * suboptimality)
    {
	    cout << "Solution cost exceeds the sub-optimality bound!" << endl;
        return false;
    }

    int actual_obj=0;
	// check whether the paths are feasible
	size_t soc = 0;
	for (int a1 = 0; a1 < num_of_agents; a1++)
	{
		soc += paths_consistent[a1]->size() - 1;
        int penalty = abs(int(paths_consistent[a1]->size()) - 1 - due_times[a1]);
        if(obj == 0){
            actual_obj=max(actual_obj,penalty);
        }
        else if(obj == 1){
            actual_obj+=penalty;
        }
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
	if (actual_obj != solution_obj)
	{
		cout << "The solution obj is wrong!" <<actual_obj<<","<<solution_obj<< endl;
		return false;
	}
	return true;
}

inline int CBSET::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths_consistent[agent_id]->size() - 1), (size_t)0);
	return paths_consistent[agent_id]->at(t);
}

// used for rapid random  restart
void CBSET::clear()
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

string CBSET::getSolverName() const
{
	string name;
	name += "CBS-ET ";
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

void CBSET::splicePaths(vector<Path> &space_paths,vector<Path> &final_paths) const
{
    int max_size=0;
    for (int i = 0; i < num_of_agents; i++)
    {
        assert(!space_paths[i].empty() && !final_paths[i].empty());
        assert(*(paths_consistent[i]->begin())==space_paths[i].back() &&
        space_paths[i].back()==final_paths[i].back());
        //拼接路径
        auto start_it = std::next(paths_consistent[i]->begin(), 1);
        space_paths[i].insert(space_paths[i].end(), start_it, paths_consistent[i]->end());
        final_paths[i].insert(final_paths[i].end(), start_it, paths_consistent[i]->end());
        //拼接final paths
        max_size=max(max_size,(int)final_paths[i].size());
    }
    for(int i=0;i<num_of_agents;++i){
        while (final_paths[i].size() < max_size) {
            // 假设补充的目标点后的部分为最后一个元素
            final_paths[i].push_back(final_paths[i].back());
        }
    }
}