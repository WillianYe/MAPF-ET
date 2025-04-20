#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

#include "CBS.h"
#include "SIPP.h"
#include "SpaceTimeAStar.h"


//从当前HLNode开始回溯到根节点，直到建立与当前HLNode一致的全部agent集合的consistent路径
// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
inline void CBS::updatePaths(CBSNode* curr)
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
}

// deep copy of all conflicts except ones that involve the particular agent
// used for copying conflicts from the parent node to the child nodes
/*void CBS::copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
	list<shared_ptr<Conflict>>& copy, int excluded_agent) const
{
	for (const auto & conflict : conflicts)
	{
		if (conflict->a1 != excluded_agent && conflict->a2 != excluded_agent)
		{
			copy.push_back(conflict);
		}
	}
}*/

// 从conflicts拷贝冲突到copy里面。一般用于从parent节点拷贝到子节点，并排除掉某些agents
void CBS::copyConflicts(const list<shared_ptr<Conflict >>& conflicts,
	list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agents)
{
	for (auto& conflict : conflicts)
	{
		bool found = false;
		for (auto a : excluded_agents)
		{
			if (conflict->a1 == a || conflict->a2 == a)
			{
				found = true;
				break;
			}
		}
		if (!found)
		{
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			copy.push_back(conflict);
		}
	}
}

// 找到HLNode上a1和a2的一致路径之间的冲突
// 冲突的初步分类：识别点、边、目标冲突。并放到unknownConf中。后续还需要classifyConflict函数进行细致检测和分类
void CBS::findConflicts(HLNode& curr, int a1, int a2)
{	
	int min_path_length = (int) (paths_consistent[a1]->size() < paths_consistent[a2]->size() ? paths_consistent[a1]->size() : paths_consistent[a2]->size());
	//在相同时间范围内的冲突，包括点冲突、目标冲突、边冲突
	for (int timestep = 0; timestep < min_path_length; timestep++)
	{
		int loc1 = paths_consistent[a1]->at(timestep);
		int loc2 = paths_consistent[a2]->at(timestep);
		if (loc1 == loc2) 
		{
			shared_ptr<Conflict> conflict(new Conflict());
			// a2更长，用到a1的goal
			if (target_reasoning && paths_consistent[a1]->size() == timestep + 1)
			{
				conflict->targetConflict(a1, a2, loc1, timestep);
			}
			// a1更长，用到a2的goal
			else if (target_reasoning && paths_consistent[a2]->size() == timestep + 1)
			{
				conflict->targetConflict(a2, a1, loc1, timestep);
			}
			else // vertex conflict
			{
				conflict->vertexConflict(a1, a2, loc1, timestep);
			}
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict);
		}
		else if (timestep < min_path_length - 1
			&& loc1 == paths_consistent[a2]->at(timestep + 1)
			&& loc2 == paths_consistent[a1]->at(timestep + 1))
		{
			shared_ptr<Conflict> conflict(new Conflict());
			conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
			assert(!conflict->constraint1.empty());
			assert(!conflict->constraint2.empty());
			curr.unknownConf.push_back(conflict); // edge conflict
		}
	}

	//在相同时间段之外的可能冲突，包括点冲突和目标冲突
	if (paths_consistent[a1]->size() != paths_consistent[a2]->size())
	{
		int shorter = paths_consistent[a1]->size() < paths_consistent[a2]->size() ? a1 : a2;
		int longer = paths_consistent[a1]->size() < paths_consistent[a2]->size() ? a2 : a1;
		int loc1 = paths_consistent[shorter]->back();
		// min_path_length 之后shorter agent永远留在目标点goal
		for (int timestep = min_path_length; timestep < (int)paths_consistent[longer]->size(); timestep++)
		{
			int loc2 = paths_consistent[longer]->at(timestep);
			if (loc1 == loc2)
			{
				shared_ptr<Conflict> conflict(new Conflict());
				if (target_reasoning)
					conflict->targetConflict(shorter, longer, loc1, timestep);
				else
					conflict->vertexConflict(shorter, longer, loc1, timestep);
				assert(!conflict->constraint1.empty());
				assert(!conflict->constraint2.empty());
				curr.unknownConf.push_front(conflict); // It's at least a semi conflict			
			}
		}
	}
}


// 得到与当前constraints一致的新的consistent路径之后，找到HLNode中的冲突
void CBS::findConflicts(HLNode& curr)
{
	clock_t t = clock();
	if (curr.parent != nullptr) //不是根节点，继承parent的冲突，并只需要对变动的agents和其他agents之间进行检测
	{
		// Copy from parent
		//首先把parent节点的conflicts拷贝过来，清理掉然后再更新涉及重规划agents的conflicts。更新到unknownConf里面
		auto new_agents = curr.getReplannedAgents();
		copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
		copyConflicts(curr.parent->unknownConf, curr.unknownConf, new_agents);

		// detect new conflicts
		for (auto it = new_agents.begin(); it != new_agents.end(); ++it)
		{
			int a1 = *it;
			for (int a2 = 0; a2 < num_of_agents; a2++)
			{
				if (a1 == a2)
					continue;
				bool skip = false;
				//不需要找new_agents之间的conflicts
				for (auto it2 = new_agents.begin(); it2 != it; ++it2)
				{
					if (*it2 == a2)
					{
						skip = true;
						break;
					}
				}
				if (!skip)
					findConflicts(curr, a1, a2);
			}
		}
	}
	else // 根节点，所有节点之间都要检测
	{
		for (int a1 = 0; a1 < num_of_agents; a1++)
		{
			for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
			{
				findConflicts(curr, a1, a2);
			}
		}
	}
	// curr.distance_to_go = (int)(curr.unknownConf.size() + curr.conflicts.size());
	runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
}

//按照conflict的优先级规则进行比较，参见重载的<比较函数。首先比较conflict_priority，再比较conflict_type，再比较second_priority
shared_ptr<Conflict> CBS::chooseConflict(const HLNode &node) const
{
	if (screen == 3)
		printConflicts(node);
	shared_ptr<Conflict> choose;
	if (node.conflicts.empty() && node.unknownConf.empty())
		return nullptr;
	else if (!node.conflicts.empty())
	{
		choose = node.conflicts.back();
		for (const auto& conflict : node.conflicts)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	else
	{
		choose = node.unknownConf.back();
		for (const auto& conflict : node.unknownConf)
		{
			if (*choose < *conflict)
				choose = conflict;
		}
	}
	return choose;
}

// 计算secondary_priority的数值。供classifyConflicts函数调用
void CBS::computeSecondPriorityForConflict(Conflict& conflict, const HLNode& node)
{
	int count[2] = {0, 0};
	switch (conflict_selection_rule)
	{
	case conflict_selection::RANDOM:
		conflict.secondary_priority = 0;
		break;
	case conflict_selection::EARLIEST:
		switch (conflict.type)
		{
		case conflict_type::STANDARD:
		case conflict_type::RECTANGLE:
		case conflict_type::TARGET:
		case conflict_type::MUTEX:
			conflict.secondary_priority = get<3>(conflict.constraint1.front());
			break;
		case conflict_type::CORRIDOR:
			conflict.secondary_priority = min(get<2>(conflict.constraint1.front()),
											  get<3>(conflict.constraint1.front()));
			break;
		default:
			break;
		}
		break;
	case conflict_selection::CONFLICTS: //当前冲突与所有冲突中发生的次数
		for (const auto& c : node.conflicts)
		{
			if (c->a1 == conflict.a1 || c->a2 == conflict.a1)
				count[0]++;
			if (c->a1 == conflict.a2 || c->a2 == conflict.a2)
				count[1]++;
		}
		conflict.secondary_priority = count[0] + count[1];
		break;
	default:
		break;
	}
}

// 方法：通过MDD
// 供classifyConflicts函数调用
//输出：CARDINAL, PSEUDO_CARDINAL, SEMI, NON
void CBS::computeConflictPriority(shared_ptr<Conflict>& con, CBSNode& node)
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
void CBS::classifyConflicts(CBSNode &node)
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
		if (con->priority == conflict_priority::CARDINAL && heuristic_helper.type == heuristics_type::ZERO)
		{
			computeSecondPriorityForConflict(*con, node);
			node.conflicts.push_back(con);
			return;
		}

		// Mutex reasoning
		if (mutex_reasoning)
		{
			// TODO mutex reasoning is per agent pair, don't do duplicated work...
			auto mdd1 = mdd_helper.getMDD(node, a1, paths_consistent[a1]->size());
			auto mdd2 = mdd_helper.getMDD(node, a2, paths_consistent[a2]->size());

			auto mutex_conflict = mutex_helper.run(a1, a2, node, mdd1, mdd2);

			if (mutex_conflict != nullptr)
			{
				computeSecondPriorityForConflict(*mutex_conflict, node);
				node.conflicts.push_back(mutex_conflict);
				continue;
			}
		}

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


void CBS::removeLowPriorityConflicts(list<shared_ptr<Conflict>>& conflicts) const
{
	if (conflicts.empty())
		return;
	unordered_map<int, shared_ptr<Conflict> > keep;
	list<shared_ptr<Conflict>> to_delete;
	for (const auto& conflict : conflicts)
	{
		int a1 = min(conflict->a1, conflict->a2), a2 = max(conflict->a1, conflict->a2);
		int key = a1 * num_of_agents + a2;
		auto p = keep.find(key);
		if (p == keep.end())
		{
			keep[key] = conflict;
		}
		else if (*(p->second) < *conflict)
		{
			to_delete.push_back(p->second);
			keep[key] = conflict;
		}
		else
		{
			to_delete.push_back(conflict);
		}
	}

	for (const auto& conflict : to_delete)
	{
		conflicts.remove(conflict);
	}
}

//检测冲突、选择冲突、生成约束之后，为约束中的新agent生成与当前节点约束一致的路径
bool CBS::findPathForSingleAgent(CBSNode*  node, int ag, int lowerbound)
{
	clock_t t = clock();
	// build reservation table
	// CAT cat(node->makespan + 1);  // initialized to false
	// updateReservationTable(cat, ag, *node);
	// find a path
	Path new_path = search_engines[ag]->findOptimalPath(*node, initial_constraints[ag], paths_consistent, ag, lowerbound);
	num_LL_expanded += search_engines[ag]->num_expanded;
	num_LL_generated += search_engines[ag]->num_generated;
	runtime_build_CT += search_engines[ag]->runtime_build_CT;
	runtime_build_CAT += search_engines[ag]->runtime_build_CAT;
	runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
	if (!new_path.empty())
	{
		assert(!isSamePath(*paths_consistent[ag], new_path));
		node->paths.emplace_back(ag, new_path);
		node->g_val = node->g_val - (int)paths_consistent[ag]->size() + (int)new_path.size();
		paths_consistent[ag] = &node->paths.back().second;
		node->makespan = max(node->makespan, new_path.size() - 1);
		return true;
	}
	else
	{
		return false;
	}
}

//把node作为parent的子节点，并检测、更新其冲突
bool CBS::generateChild(CBSNode*  node, CBSNode* parent)
{
	clock_t t1 = clock();
	node->parent = parent;
	node->HLNode::parent = parent;
	node->g_val = parent->g_val;
	node->makespan = parent->makespan;
	node->depth = parent->depth + 1;

	auto agents = getInvalidAgents(node->constraints);
	assert(!agents.empty());
	for (auto agent : agents)
	{
		int lowerbound = (int)paths_consistent[agent]->size() - 1;
		//找不到新路径
		if (!findPathForSingleAgent(node, agent, lowerbound))
		{
			runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
			return false;
		}
	}

	findConflicts(*node);
	heuristic_helper.computeQuickHeuristics(*node);
	runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
	return true;
}

inline void CBS::pushNode(CBSNode* node)
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


inline bool CBS::reinsertNode(CBSNode* node)
{
    node->cleanup_handle = cleanup_list.push(node);
	switch (solver_type)
	{
	case high_level_solver_type::ASTAR:
		break;
	case high_level_solver_type::ASTAREPS:
		if (node->getFVal() <= suboptimality * obj_lowerbound)
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
	if (screen == 2)
	{
		cout << "	Reinsert " << *node << endl;
	}
	return true;
}

//根据A×，FS(f=g)，FS(f=g+h)，EES寻找顶部节点
CBSNode* CBS::selectNode()
{
	CBSNode* curr = nullptr;
	switch (solver_type)
	{
        case high_level_solver_type::ASTAR:
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
	updatePaths(curr);

	if (screen > 1)
		cout << endl << "Pop " << *curr << endl;
	return curr;
}

// return agents that violate the constraints
// 输入：节点中的新约束
// 输出：如果是STANDARD约束，则直接输出约束中相对应的agent；
// 如果是其他类型约束，如LEQLENGTH，POSITIVE_VERTEX，POSITIVE_EDGE，则找到更多关联的agent
// 这是因为，这三种约束会改变其他agents的plans
set<int> CBS::getInvalidAgents(const list<Constraint>& constraints)
{
	set<int> agents;
	int agent, x, y, t;
	constraint_type type;
	assert(!constraints.empty());
	tie(agent, x, y, t, type) = constraints.front();

	if (type == constraint_type::LEQLENGTH)
	{
		assert(constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			// 当前agent在t时刻之前到达点x
			if (ag == agent)
				continue;
			// t时刻之后到达x的agents都非法
			for (int i = t; i < (int)paths_consistent[ag]->size(); i++)
			{
				if (paths_consistent[ag]->at(i) == x)
				{
					agents.insert(ag);
					break;
				}
			}
		}
	}
	else if (type == constraint_type::POSITIVE_VERTEX)
	{
		assert(constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			// 当前agent在t时刻位于点x
			if (ag == agent)
				continue;
			// t时刻位于x的agents都非法
			if (getAgentLocation(ag, t) == x)
			{
				agents.insert(ag);
			}
		}
	}
	else if (type == constraint_type::POSITIVE_EDGE)
	{
		assert(constraints.size() == 1);
		for (int ag = 0; ag < num_of_agents; ag++)
		{
			if (ag == agent)
				continue;
			int curr = getAgentLocation(ag, t);
			int prev = getAgentLocation(ag, t - 1);
			if (prev == x || curr == y ||
				(prev == y && curr == x))
			{
				agents.insert(ag);
			}
		}

	}
	else //STANDARD constraint
	{
		agents.insert(agent);
	}
	return agents;
}


void CBS::printPaths() const
{
	for (int i = 0; i < num_of_agents; i++)
	{
		cout << "Agent " << i << " (length " << paths_found_initially[i].size() - 1 << " -->" <<
			paths_consistent[i]->size() - 1 << "): ";
		for (const auto & t : *paths_consistent[i])
			cout << t << "->";
		cout << endl;
	}
}



void CBS::printResults() const
{
	if (solution_cost >= 0) // solved
		cout << "Succeed,";
	else if (solution_cost == -1) // time_out
		cout << "Timeout,";
	else if (solution_cost == -2) // no solution
		cout << "No solutions,";
	else if (solution_cost == -3) // nodes out
		cout << "Nodesout,";

	cout << solution_cost << "," << runtime << "," <<
		num_HL_expanded << "," << num_LL_expanded << "," << // HL_num_generated << "," << LL_num_generated << "," <<
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

void CBS::saveResults(const string &fileName, const string &instanceName) const
{
	std::ifstream infile(fileName);
	bool exist = infile.good();
	infile.close();
	if (!exist)
	{
		ofstream addHeads(fileName);
		addHeads << "runtime,#high-level expanded,#high-level generated,#low-level expanded,#low-level generated," <<
			"solution cost,min f value,root g value, root f value," <<
			"#adopt bypasses," <<
			"cardinal conflicts," <<
			"standard conflicts,rectangle conflicts,corridor conflicts,target conflicts,mutex conflicts," <<
			"chosen from cleanup,chosen from open,chosen from focal," <<
			"#solve MVCs,#merge MDDs,#solve 2 agents,#memoization," <<
			"cost error,distance error," <<
			"runtime of building heuristic graph,runtime of solving MVC," <<
			"runtime of detecting conflicts," <<
			"runtime of rectangle conflicts,runtime of corridor conflicts,runtime of mutex conflicts," <<
			"runtime of building MDDs,runtime of building constraint tables,runtime of building CATs," <<
			"runtime of path finding,runtime of generating child nodes," <<
			"preprocessing runtime,solver name,instance name" << endl;
		addHeads.close();
	}
	ofstream stats(fileName, std::ios::app);
	stats << runtime << "," << 
		num_HL_expanded << "," << num_HL_generated << "," <<
		num_LL_expanded << "," << num_LL_generated << "," <<

		solution_cost << "," << obj_lowerbound << "," << dummy_start->g_val << "," <<
		dummy_start->g_val + dummy_start->h_val << "," <<

		num_adopt_bypass << "," <<
		num_cardinal_conflicts << "," <<
		num_standard_conflicts << "," << num_rectangle_conflicts << "," << num_corridor_conflicts << "," << num_target_conflicts << "," << num_mutex_conflicts << "," <<

		num_cleanup << "," << num_open << "," << num_focal << "," <<

		heuristic_helper.num_solve_MVC << "," <<
		heuristic_helper.num_merge_MDDs << "," << 
		heuristic_helper.num_solve_2agent_problems << "," << 
		heuristic_helper.num_memoization << "," <<
		heuristic_helper.getCostError() << "," << heuristic_helper.getDistanceError() << "," <<
		heuristic_helper.runtime_build_dependency_graph << "," << 
		heuristic_helper.runtime_solve_MVC << "," <<

		

		runtime_detect_conflicts << "," << 
		rectangle_helper.accumulated_runtime << "," << corridor_helper.accumulated_runtime << "," << mutex_helper.accumulated_runtime << "," <<
		mdd_helper.accumulated_runtime << "," << runtime_build_CT << "," << runtime_build_CAT << "," <<
		runtime_path_finding << "," << runtime_generate_child << "," <<

		runtime_preprocessing << "," << getSolverName() << "," << instanceName << endl;
	stats.close();
}

void CBS::saveStats(const string &fileName, const string &instanceName)
{
	/*cout << "writing logs..." << endl;
	ofstream stats(fileName+ ".txt", std::ios::app);
	stats << instanceName << endl;
	stats << "agent 1,agent 2,node id,#expanded nodes, h" << endl;
	for (auto ins : heuristic_helper.sub_instances)
	{
			stats << get<0>(ins) << "," << get<1>(ins) << "," << get<2>(ins)->time_generated << "," << get<3>(ins) << "," << get<4>(ins) << endl;
	}
	stats.close();*/
	/*if (solution_found)
	{
		std::ifstream infile(fileName + ".heuristic");
		bool exist = infile.good();
		infile.close();
		if (!exist)
		{
			ofstream addHeads(fileName + ".heuristic");
			addHeads << "h*,solution depth,WDG,greedy WDG,DG,MVC on all conflicts,#conflicts,#CT nodes" << endl;
			addHeads.close();
		}
		ofstream out(fileName + ".heuristic", std::ios::app);
		out << solution_cost - dummy_start->g_val << "," // h*
			<< goal_node->depth << "," // depth
			<< dummy_start->h_val << ","; // WDG
		updatePaths(dummy_start);
		findConflicts(*dummy_start);
		classifyConflicts(*dummy_start);
		// int greedy = heuristic_helper.greedyWDG(*dummy_start, time_limit);
		int greedy = 0;
		heuristic_helper.type = heuristics_type::DG;
		int dg = heuristic_helper.computeInformedHeuristics(*dummy_start, time_limit);
		// int mvc = heuristic_helper.MVConAllConflicts(*dummy_start);
		int mvc = 0;
		out << greedy << "," << dg << "," << mvc << "," << dummy_start->conflicts.size() << "," << 
			dummy_start->distance_to_go << endl;
		out.close();
	}*/
}

void CBS::saveCT(const string &fileName) const // write the CT to a file
{
	// Write the tree graph in dot language to a file
	{
		std::ofstream output;
		output.open(fileName + ".tree", std::ios::out);
		output << "digraph G {" << endl;
		output << "size = \"5,5\";" << endl;
		output << "center = true;" << endl;
		set<HLNode*> path_to_goal;
		auto curr = goal_node;
		while (curr != nullptr)
		{
			path_to_goal.insert(curr);
			curr = curr->parent;
		}
		for (const auto& node : allNodes_table)
		{
			output << node->time_generated << " [label=\"g=" << node->g_val << ", h=" << node->h_val
				<< "\nd=" << node->distance_to_go << ", h^=" << node->getFHatVal() - node->g_val;
			if (node->time_expanded > 0) // the node has been expanded
			{
				output << "\n #" << node->time_expanded << " from " << node->chosen_from;
				output << "\", color=";
				if (node->chosen_from == "focal")
					output << "blue]" << endl;
				else if (node->chosen_from == "cleanup")
					output << "green]" << endl;
				else if (node->chosen_from == "open")
					output << "orange]" << endl;
			}
			else
			{
				output << "\"]" << endl;
			}

			if (node == dummy_start)
				continue;
			if (path_to_goal.find(node) == path_to_goal.end())
			{
				output << node->parent->time_generated << " -> " << node->time_generated << endl;
			}
			else
			{
				output << node->parent->time_generated << " -> " << node->time_generated << " [color=red]" << endl;
			}
		}
		output << "}" << endl;
		output.close();
	}
	
	// Write the stats of the tree to a CSV file
	{
		std::ofstream output;
		output.open(fileName + "-tree.csv", std::ios::out);
		// header
		output << "time generated,g value,h value,h^ value,d value,depth,time expanded,chosen from,h computed," 
			<< "f of best in cleanup,f^ of best in cleanup,d of best in cleanup," 
			<< "f of best in open,f^ of best in open,d of best in open," 
			<< "f of best in focal,f^ of best in focal,d of best in focal,"
			<< "praent,goal node" << endl;
		for (auto& node : allNodes_table)
		{
			output << node->time_generated << ","
				<< node->g_val << "," << node->h_val << "," << node->getFHatVal() - node->g_val << "," <<  node->distance_to_go << ","
				<< node->depth << ","
				<< node->time_expanded << "," << node->chosen_from << "," << node->h_computed << ","
				<< node->f_of_best_in_cleanup << "," << node->f_hat_of_best_in_cleanup << "," << node->d_of_best_in_cleanup << ","
				<< node->f_of_best_in_open << "," << node->f_hat_of_best_in_open << "," << node->d_of_best_in_open << ","
				<< node->f_of_best_in_focal << "," << node->f_hat_of_best_in_focal << "," << node->d_of_best_in_focal << ",";
			if (node->parent == nullptr)
				output << "0,";
			else
				output << node->parent->time_generated << ",";
			if (node == goal_node)
				output << "1" << endl;
			else
				output << "0" << endl;
		}
		output.close();
	}

}

void CBS::savePaths(const string &fileName) const
{
    std::ofstream output;
    output.open(fileName, std::ios::out);
    for (int i = 0; i < num_of_agents; i++)
    {
        output << i << ":";
        for (const auto & t : *paths_consistent[i])
            output << "(" << search_engines[0]->instance.getRowCoordinate(t)
                   << "," << search_engines[0]->instance.getColCoordinate(t) << "),";
        output << endl;
    }
    output.close();
}

void CBS::printConflicts(const HLNode &curr)
{
	for (const auto& conflict : curr.conflicts)
	{
		cout << *conflict << endl;
	}
	for (const auto& conflict : curr.unknownConf)
	{
		cout << *conflict << endl;
	}
}


string CBS::getSolverName() const
{
	string name;
	if (disjoint_splitting)
		name += "Disjoint ";
	switch (heuristic_helper.type)
	{
	case heuristics_type::ZERO:
		if (PC)
			name += "ICBS";
		else
			name += "CBS";
		break;
	case heuristics_type::CG:
		name += "CG";
		break;
	case heuristics_type::DG:
		name += "DG";
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
	name += " with " + search_engines[0]->getName();
	return name;
}


bool CBS::solve(double _time_limit, int _cost_lowerbound, int _cost_upperbound)
{
	this->obj_lowerbound = _cost_lowerbound;
	this->inadmissible_obj_lowerbound = 0;
	this->cost_upperbound = _cost_upperbound;
	this->time_limit = _time_limit;

	if (screen > 0) // 1 or 2
	{
		string name = getSolverName();
		name.resize(35, ' ');
		cout << name << ": ";
	}
	// set timer
	start = clock();

	generateRoot();

	while (!cleanup_list.empty() && !solution_found)
	{
		auto curr = selectNode();

		if (terminate(curr))
			return solution_found;

		if (PC) // priortize conflicts
			classifyConflicts(*curr);

		if (!curr->h_computed) // heuristics has not been computed yet
		{
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
			bool succ = heuristic_helper.computeInformedHeuristics(*curr, time_limit - runtime);
			runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
            heuristic_helper.updateOnlineHeuristicErrors(*curr);
            heuristic_helper.updateInadmissibleHeuristics(*curr); // compute inadmissible heuristics
			/*if (runtime > time_limit)
			{  // timeout
				solution_cost = -1;
				solution_found = false;
				break;
			}*/
			if (!succ) // no solution, so prune this node
			{
				curr->clear();
				continue;
			}

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
			CBSNode* child[2] = { new CBSNode() , new CBSNode() };

			curr->conflict = chooseConflict(*curr);

			addConstraints(curr, child[0], child[1]);

			if (screen > 1)
				cout << "	Expand " << *curr << endl <<
				"	on " << *(curr->conflict) << endl;

			bool solved[2] = { false, false };
			vector<vector<int>*> copy(paths_consistent);

			for (int i = 0; i < 2; i++)
			{
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
					if (i == 1 && !solved[0])
						continue;
					foundBypass = true;
					num_adopt_bypass++;
					curr->conflicts = child[i]->conflicts;
					curr->unknownConf = child[i]->unknownConf;
					curr->distance_to_go = child[i]->distance_to_go;
					curr->conflict = nullptr;
					for (const auto& path : child[i]->paths) // update paths
					{
						auto p = curr->paths.begin();
						while (p != curr->paths.end())
						{
							if (path.first == p->first)
							{
								p->second = path.second;
								paths_consistent[p->first] = &p->second;
								break;
							}
							++p;
						}
						if (p == curr->paths.end())
						{
							curr->paths.emplace_back(path);
							paths_consistent[path.first] = &curr->paths.back().second;
						}
					}
					if (screen > 1)
					{
						cout << "	Update " << *curr << endl;
					}
					break;
				}
			}
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
			else
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
bool CBS::terminate(HLNode* curr)
{
	if (obj_lowerbound >= cost_upperbound)
	{
		solution_cost = obj_lowerbound;
		solution_found = false;
        if (screen > 0) // 1 or 2
            printResults();
		return true;
	}
	runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
	if (curr->conflicts.empty() && curr->unknownConf.empty()) //no conflicts
	{// found a solution
		solution_found = true;
		goal_node = curr;
		solution_cost = goal_node->getFHatVal() - goal_node->cost_to_go;
		if (!validateSolution())
		{
			cout << "Solution invalid!!!" << endl;
			printPaths();
			exit(-1);
		}
		if (screen > 0) // 1 or 2
			printResults();
		return true;
	}
	if (runtime > time_limit || num_HL_expanded > node_limit)
	{   // time/node out
		solution_cost = -1;
		solution_found = false;
        if (screen > 0) // 1 or 2
            printResults();
		return true;
	}
	return false;
}

// 把parent节点的约束施加到子节点
void CBS::addConstraints(const HLNode* curr, HLNode* child1, HLNode* child2) const
{
	// disjoint splitting只针对标准冲突类型
	if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
	{
		// rand()函数不需要参数,它会返回一个从 0 到最大随机数之间的任意整数
		int first = (bool)(rand() % 2);
		if (first) // disjoint splitting on the first agent
		{
			child1->constraints = curr->conflict->constraint1; // is a list
			int a, x, y, t;
			constraint_type type;
			tie(a, x, y, t, type) = curr->conflict->constraint1.back();
			if (type == constraint_type::VERTEX)
			{
				child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
			}
			else
			{
				assert(type == constraint_type::EDGE);
				child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
			}
		}
		else // disjoint splitting on the second agent
		{
			child2->constraints = curr->conflict->constraint2;
			int a, x, y, t;
			constraint_type type;
			tie(a, x, y, t, type) = curr->conflict->constraint2.back();
			if (type == constraint_type::VERTEX)
			{
				child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
			}
			else
			{
				assert(type == constraint_type::EDGE);
				child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
			}
		}
	}
	else
	{
		child1->constraints = curr->conflict->constraint1;
		child2->constraints = curr->conflict->constraint2;
	}
}

CBS::CBS(vector<SingleAgentSolver*>& search_engines,
	const vector<ConstraintTable>& initial_constraints,
	vector<Path>& paths_found_initially, int screen) :
	screen(screen), suboptimality(1), 
	initial_constraints(initial_constraints), paths_found_initially(paths_found_initially),
	search_engines(search_engines), 
	mdd_helper(initial_constraints, search_engines),
	rectangle_helper(search_engines[0]->instance),
	mutex_helper(search_engines[0]->instance, initial_constraints),
	corridor_helper(search_engines, initial_constraints),
	heuristic_helper(search_engines.size(), paths_consistent, search_engines, initial_constraints, mdd_helper)
{
	num_of_agents = (int) search_engines.size();
	mutex_helper.search_engines = search_engines;
}

CBS::CBS(const Instance& instance, bool sipp, int screen) :
	screen(screen), suboptimality(1),
	num_of_agents(instance.getDefaultNumberOfAgents()),
	mdd_helper(initial_constraints, search_engines),
	rectangle_helper(instance),
	mutex_helper(instance, initial_constraints),
	corridor_helper(search_engines, initial_constraints),
	heuristic_helper(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper)
{
	clock_t t = clock();
	initial_constraints.resize(num_of_agents, 
		ConstraintTable(instance.num_of_cols, instance.map_size));

	search_engines.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (sipp)
			search_engines[i] = new SIPP(instance, i);
		else
			search_engines[i] = new SpaceTimeAStar(instance, i);
	}
	runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

	mutex_helper.search_engines = search_engines;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

CBS::CBS(const Instance& instance, const vector<ConstraintTable>& initial_constraints, bool sipp, int screen) :
	screen(screen), suboptimality(1),
	num_of_agents(instance.getDefaultNumberOfAgents()),
	initial_constraints(initial_constraints),
	mdd_helper(initial_constraints, search_engines),
	rectangle_helper(instance),
	mutex_helper(instance, initial_constraints),
	corridor_helper(search_engines, initial_constraints),
	heuristic_helper(instance.getDefaultNumberOfAgents(), paths_consistent, search_engines, initial_constraints, mdd_helper)
{
	clock_t t = clock();
	search_engines.resize(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		if (sipp)
			search_engines[i] = new SIPP(instance, i);
		else
			search_engines[i] = new SpaceTimeAStar(instance, i);
	}
	runtime_preprocessing = (double)(clock() - t) / CLOCKS_PER_SEC;

	mutex_helper.search_engines = search_engines;

	if (screen >= 2) // print start and goals
	{
		instance.printAgents();
	}
}

//generate random permutation of agent indices
vector<int> CBS::shuffleAgents() const
{
	vector<int> agents(num_of_agents);
	for (int i = 0; i < num_of_agents; i++)
	{
		agents[i] = i;
	}

	if (randomRoot)
	{
		std::random_device rd;
		std::mt19937 g(rd());
		std::shuffle(std::begin(agents), std::end(agents), g);
	}
	return agents;
}

bool CBS::generateRoot()
{
	auto root = new CBSNode();
	root->g_val = 0;
	paths_consistent.resize(num_of_agents, nullptr);

	mdd_helper.init(num_of_agents);
	heuristic_helper.init();

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

			paths_found_initially[i] = search_engines[i]->findOptimalPath(*root, initial_constraints[i], paths_consistent, i, 0);
			if (paths_found_initially[i].empty())
			{
				if (screen >= 2)
					cout << "No path exists for agent " << i << endl;
				delete root;
				return false;
			}
			paths_consistent[i] = &paths_found_initially[i];			
			root->g_val += (int)paths_found_initially[i].size() - 1;
			root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
			num_LL_expanded += search_engines[i]->num_expanded;
			num_LL_generated += search_engines[i]->num_generated;
		}
	}
	else
	{
		for (int i = 0; i < num_of_agents; i++)
		{
			paths_consistent[i] = &paths_found_initially[i];
			root->makespan = max(root->makespan, paths_found_initially[i].size() - 1);
			root->g_val += (int) paths_found_initially[i].size() - 1;
		}
	}

	root->h_val = 0;
	root->depth = 0;
	findConflicts(*root);
	heuristic_helper.computeQuickHeuristics(*root);
	pushNode(root);
	dummy_start = root;
	if (screen >= 2) // print start and goals
	{
		printPaths();
	}

	return true;
}

inline void CBS::releaseNodes()
{
	open_list.clear();
	cleanup_list.clear();
	focal_list.clear();
	for (auto& node : allNodes_table)
		delete node;
	allNodes_table.clear();
}

CBS::~CBS()
{
	releaseNodes();
	mdd_helper.clear();
}

// driver中会用到
void CBS::clearSearchEngines()
{
	for (auto s : search_engines)
		delete s;
	search_engines.clear();
}

bool CBS::validateSolution() const
{
	// check whether the solution cost is within the bound
	if (solution_cost > obj_lowerbound * suboptimality)
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

inline int CBS::getAgentLocation(int agent_id, size_t timestep) const
{
	size_t t = max(min(timestep, paths_consistent[agent_id]->size() - 1), (size_t)0);
	return paths_consistent[agent_id]->at(t);
}

// used for rapid random  restart
void CBS::clear()
{
	mdd_helper.clear();
	heuristic_helper.clear();
	releaseNodes();
	paths_consistent.clear();
	paths_found_initially.clear();
	dummy_start = nullptr;
	goal_node = nullptr;
	solution_found = false;
	solution_cost = -2;
}
