#include "SpaceTimeAStarET.h"

Path SpaceTimeAStarET::findOptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
                                    const vector<Path*>& paths, int agent, int lowerbound)
{
    return findSuboptimalPath(node, initial_constraints, paths, agent, lowerbound, 1).first;
}

pair<Path, int> SpaceTimeAStarET::findSuboptimalPath(const HLNode& node, const ConstraintTable& initial_constraints,
    const vector<Path*>& paths, int agent, int lower_bound, double w)
{
    this->w = w;
    Path path;
    num_expanded = 0;
    num_generated = 0;

    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(initial_constraints);
    constraint_table.insert2CT(node, agent);
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;
    if (constraint_table.constrained(start_location, 0))
    {
        return {path,0};
    }

    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    //用于切换到纯空间Astar
    //auto static_timestep = constraint_table.getMaxTimestep() + 1; // everything is static after this timestep
    auto static_timestep = max(due_time,constraint_table.getMaxTimestep()) + 1; // everything is static after this timestep

    // the earliest timestep that the agent can hold its goal location. The length_min is considered here.
    auto holding_time = constraint_table.getHoldingTime(goal_location, constraint_table.length_min);

    // lowerbound更新中，holding_time是可容许的吗？
    lower_bound =  max(holding_time, lower_bound);

    // generate start and add it to the OPEN list
    auto start = new AStarNode(start_location, 0, max(lower_bound, my_heuristic[start_location]), nullptr, 0, 0);

    num_generated++;
    // 初始点压入open
    start->open_handle = open_list.push(start);
    start->focal_handle = focal_list.push(start);
    start->in_openlist = true;
    allNodes_table.insert(start);
    min_f_val = start->getFVal();

    int upper_bound = constraint_table.length_max;
    AStarNode* current_best = nullptr;

    while (!open_list.empty())
    {
        updateFocalList();
        auto* curr = popNode();
        assert(curr->location >= 0);

        //如果curr最后是某条路径上的点，那么这个路径的cost>=f(curr)>upper_bound，排除
        if (curr->timestep + curr->h_val > upper_bound)
            continue;

        // check if the popped node is a goal
        if (curr->location == goal_location && curr->timestep >= holding_time) // the agent can not hold the goal location afterward
        {
            if(curr->timestep >= due_time){
                current_best = curr;
                break;
            }
            if(current_best == nullptr || curr->timestep > current_best->timestep){
                current_best = curr;
                lower_bound = curr->timestep + 1;
                upper_bound = 2*due_time - curr->timestep - 1;
            }
            //视为第一次进入后就不能出来
            continue;
        }
        //邻居不包含当前点
        auto next_locations = instance.getNeighbors(curr->location);
        next_locations.emplace_back(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep + 1;
            if (static_timestep < next_timestep)
            { // now everything is static, so switch to space A* where we always use the same timestep
                if (next_location == curr->location)
                {
                    continue;
                }
                next_timestep--;
            }
            if (constraint_table.constrained(next_location, next_timestep) ||
                constraint_table.constrained(curr->location, next_location, next_timestep))
                continue;
            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            //同时利用传入的启发信息和建立的启发信息
            int next_h_val = max(lower_bound - next_g_val, my_heuristic[next_location]);
            if (next_g_val + next_h_val > upper_bound)
                continue;
            int next_internal_conflicts = curr->num_of_conflicts +
                                        constraint_table.getNumOfConflictsForStep(curr->location, next_location, next_timestep);

            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                    curr, next_timestep, next_internal_conflicts);
            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            //若为新节点，则加入open
            if (it == allNodes_table.end())
            {
                pushNode(next);
                allNodes_table.insert(next);
                continue;
            }
            auto existing_next = *it; //*为解引用符
            if (existing_next->getFVal() > next->getFVal() || // if f-val decreased through this new path
                (existing_next->getFVal() == next->getFVal() &&
                existing_next->num_of_conflicts > next->num_of_conflicts)) // or it remains the same but there's fewer conflicts
            {
                if (!existing_next->in_openlist) // if it is in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    pushNode(existing_next);
                }
                else
                {
                    bool add_to_focal = false;  // check if it was above the focal bound before and now below (thus need to be inserted)
                    bool remove_from_focal = false;  // check if it was inside the focal and now above the bound (thus need to be deleted)
                    bool update_in_focal = false;  // check if it was inside the focal and needs to be updated (because f-val changed)
                    bool update_open = false;
                    if (abs(next->getFVal()-due_time) <= w * abs(min_f_val-due_time))
                    {  // if the new f-val qualify to be in FOCAL
                        if (abs(existing_next->getFVal()-due_time) > w * abs(min_f_val-due_time))
                            add_to_focal = true;  // and the previous f-val did not qualify to be in FOCAL then add
                        else
                            update_in_focal = true;  // and the previous f-val did qualify to be in FOCAL then update
                    }
                    else
                    {  // if the new f-val does not qualify to be in FOCAL
                        if (abs(existing_next->getFVal()-due_time) <= w * abs(min_f_val-due_time))
                            remove_from_focal = true; // and the previous f-val qualified to be in FOCAL then remove
                    }
                    if (existing_next->getFVal() > next->getFVal())
                        update_open = true;

                    existing_next->copy(*next);	// update existing node

                    if (update_open){
                        open_list.increase(existing_next->open_handle);  // increase because f-val improved
                    }  
                    if (add_to_focal){
                        existing_next->focal_handle = focal_list.push(existing_next);
                    }
                    if (update_in_focal){
                        focal_list.update(existing_next->focal_handle);  // should we do update? yes, because number of conflicts may go up or down
                    }
                    if (remove_from_focal){
                        focal_list.erase(existing_next->focal_handle);
                    }             
                }
            }
            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop
    if(current_best){
        updatePath(current_best, path);
    }
    releaseNodes();
    return {path, min_f_val};
}


Path SpaceTimeAStarET::findOptimalPath(const unordered_set<int>&forbid_locations){
    Path path;
    num_expanded = 0;
    num_generated = 0;
    replan_times ++;
    auto start_t = clock();
    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, my_heuristic[start_location], nullptr, 0, 0);

    num_generated++;
    replan_generated_num++;
    start->open_handle = open_list.push(start);

    start->in_openlist = true;
    allNodes_table.insert(start);

    while (!open_list.empty())
    {
        auto* curr = open_list.top(); 
        open_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        replan_expanded_num++;

        // check if the popped node is a goal
        if (curr->location == goal_location)
        {
            updatePath(curr, path);
            replan_success_times ++;
            break;
        }

        auto next_locations = instance.getNeighbors(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep+1;
            
            if (forbid_locations.find(next_location)!=forbid_locations.end())
                continue;            

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            int next_h_val = my_heuristic[next_location];

            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, 0);


            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                num_generated++;
                replan_generated_num++;
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal())
            {
                if (!existing_next->in_openlist) // if it is in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->open_handle = open_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++;
                    replan_generated_num++;
                }
                else
                {
                    // bool update_open = (existing_next->getFVal() > next_g_val + next_h_val);
                    // existing_next->copy(*next);	// update existing node
                    // if (update_open)
                    //     open_list.increase(existing_next->open_handle);  // increase because f-val improved
                    existing_next->copy(*next);
                    open_list.increase(existing_next->open_handle);
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    open_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
    return path;
}

Path SpaceTimeAStarET::findOptimalPath(const unordered_map<pair<int,int>,int> &edge_heat_map,
const unordered_set<int>&forbid_locations)
{
    Path path;
    num_expanded = 0;
    num_generated = 0;
    replan_times ++;
    auto start_t = clock();
    // generate start and add it to the OPEN & FOCAL list
    auto start = new AStarNode(start_location, 0, my_heuristic[start_location], nullptr, 0, 0);

    num_generated++;
    replan_generated_num++;
    start->open_handle = open_list.push(start);

    start->in_openlist = true;
    allNodes_table.insert(start);

    while (!open_list.empty())
    {
        auto* curr = open_list.top(); 
        open_list.pop();
        curr->in_openlist = false;
        num_expanded++;
        replan_expanded_num++;

        // check if the popped node is a goal
        if (curr->location == goal_location)
        {
            updatePath(curr, path);
            replan_success_times ++;
            break;
        }

        auto next_locations = instance.getNeighbors(curr->location);
        for (int next_location : next_locations)
        {
            int next_timestep = curr->timestep+1;
            
            if (forbid_locations.find(next_location)!=forbid_locations.end())
                continue;            

            // compute cost to next_id via curr node
            int next_g_val = curr->g_val + 1;
            auto hm_it= edge_heat_map.find(make_pair(curr->location,next_location));
            if(hm_it!=edge_heat_map.end()){
                next_g_val+=hm_it->second;
            }
            int next_h_val = my_heuristic[next_location];
            // generate (maybe temporary) node
            auto next = new AStarNode(next_location, next_g_val, next_h_val,
                                      curr, next_timestep, 0);


            // try to retrieve it from the hash table
            auto it = allNodes_table.find(next);
            if (it == allNodes_table.end())
            {
                next->open_handle = open_list.push(next);
                next->in_openlist = true;
                num_generated++;
                replan_generated_num++;
                allNodes_table.insert(next);
                continue;
            }
            // update existing node's if needed (only in the open_list)

            auto existing_next = *it;
            if (existing_next->getFVal() > next->getFVal())
            {
                if (!existing_next->in_openlist) // if it is in the closed list (reopen)
                {
                    existing_next->copy(*next);
                    existing_next->open_handle = open_list.push(existing_next);
                    existing_next->in_openlist = true;
                    num_generated++;
                    replan_generated_num++;
                }
                else
                {
                    // bool update_open = (existing_next->getFVal() > next_g_val + next_h_val);
                    // existing_next->copy(*next);	// update existing node
                    // if (update_open)
                    //     open_list.increase(existing_next->open_handle);  // increase because f-val improved
                    existing_next->copy(*next);
                    open_list.increase(existing_next->open_handle);
                }
            }

            delete(next);  // not needed anymore -- we already generated it before
        }  // end for loop that generates successors
    }  // end while loop

    open_list.clear();
    for (auto node: allNodes_table)
        delete node;
    allNodes_table.clear();
    return path;
}

inline void SpaceTimeAStarET::pushNode(AStarNode* node)
{
    node->open_handle = open_list.push(node);
    node->in_openlist = true;
    num_generated++;
    if (abs(node->getFVal()-due_time) <= w * abs(min_f_val-due_time)){
        node->focal_handle = focal_list.push(node);
    }
        
}

inline AStarNode* SpaceTimeAStarET::popNode()
{
    auto node = focal_list.top(); focal_list.pop();
    if(node->in_openlist){
        open_list.erase(node->open_handle);
    }
    node->in_openlist = false;
    num_expanded++;
    return node;
}

void SpaceTimeAStarET::updateFocalList()
{
    auto open_head = open_list.top();
    int new_f_val_sub = abs(open_head->getFVal()-due_time);
    int f_val_sub = abs(min_f_val-due_time);
    if (new_f_val_sub > f_val_sub)
    {      
        for (auto n : open_list)
        {
            if (abs(n->getFVal() - due_time) >  w * f_val_sub && 
            abs(n->getFVal() - due_time) <= w * new_f_val_sub){
                n->focal_handle = focal_list.push(n);
            }      
        }
        min_f_val = open_head->getFVal();
    }
}