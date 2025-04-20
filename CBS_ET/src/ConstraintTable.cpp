#include "ConstraintTable.h"

// return the earliest timestep that the agent can hold the location
// 返回可以永久占用location的最早时间，而不是可暂留的时间
int ConstraintTable::getHoldingTime(int location, int earliest_timestep) const
{
    int rst = earliest_timestep;
    // CT
    auto it = ct.find(location);
    if (it != ct.end())
    {
        for (auto time_range : it->second)
            rst = max(rst, time_range.second);
    }
    // Landmark
    // 若存在landmarks，因为必须要在某时刻到达landmarks上，所以rst必定要大于其值
    for (auto landmark : landmarks)
    {
        if (landmark.second != location)
            rst = max(rst, (int)landmark.first + 1);
    }

    return rst;
}

// 相当于规划到这个时间长度
int ConstraintTable::getMaxTimestep() const // everything is static after the max timestep
{
    int rst = max(max(ct_max_timestep, cat_max_timestep), length_min);
    if (length_max < MAX_TIMESTEP)
        rst = max(rst, length_max);
    // landmarks是有序的map，只需要比较最后的值
    if (!landmarks.empty())
        rst = max(rst, landmarks.rbegin()->first);
    return rst;
}

// 机器人最大的冲突时间
int ConstraintTable::getLastCollisionTimestep(int location) const
{
    int rst = -1;
    if (!cat.empty())
    {
        for (auto t = cat[location].size() - 1; t > rst; t--)
        {
            if (cat[location][t])
                return t;
        }
    }
    return rst;
}

//插入点或边冲突到CT
void ConstraintTable::insert2CT(size_t loc, int t_min, int t_max)
{
    assert(loc >= 0);
    ct[loc].emplace_back(t_min, t_max);
    if (t_max < MAX_TIMESTEP && t_max > ct_max_timestep)
    {
        ct_max_timestep = t_max;
    }
    //时间点
    else if (t_max == MAX_TIMESTEP && t_min > ct_max_timestep)
    {
        ct_max_timestep = t_min;
    }
}

// 插入边冲突到CT
void ConstraintTable::insert2CT(size_t from, size_t to, int t_min, int t_max)
{
    insert2CT(getEdgeIndex(from, to), t_min, t_max);
}

//插入约束列表。
// add constraints for the given agent
// typedef std::tuple<int, int, int, int, constraint_type> Constraint
void ConstraintTable::insert2CT(const list<Constraint>& constraints, int agent)
{
    if (constraints.empty())
        return;
    int a, x, y, t;
    constraint_type type;
    // std::tie 可以用于解包 tuple 和 pair，因为 std::tuple 拥有从 pair 的转换赋值。
    // std::tie 可以将多个变量的引用整合成一个 tuple，进而通过另外一个同类型的 tuple 进行批量赋值。
    // 输入constraints中的agent均一致。因此只需要判断第一个元素是不是agent
    tie(a, x, y, t, type) = constraints.front();
    
    switch (type)
    {
        //路径长度应该小于某值
        case constraint_type::LEQLENGTH:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to reach its goal at or before timestep t.
                length_max = min(length_max, t); //缩小最大长度的bound
            else // other agents cannot stay at x at or after timestep t
                // 其他agent永远占用他们的goal目标点。给本agent施加约束
                insert2CT(x, t, MAX_TIMESTEP);
            break;
        //路径长度应该大于某值
        case constraint_type::GLENGTH:
            assert(constraints.size() == 1);
            if (a == agent) // path of agent_id should be of length at least t + 1
                length_min = max(length_min, t + 1); //放大最小长度的bound
            break;
        case constraint_type::POSITIVE_VERTEX:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to be at x at timestep t
            {
                insertLandmark(x, t);
            }
            else // other agents cannot stay at x at timestep t
            {
                insert2CT(x, t, t + 1);
            }
            break;
        case constraint_type::POSITIVE_EDGE:
            assert(constraints.size() == 1);
            if (agent == a) // this agent has to be at x at timestep t - 1 and be at y at timestep t
            {
                insertLandmark(x, t - 1);
                insertLandmark(y, t);
            }
            else // other agents cannot stay at x at timestep t - 1, be at y at timestep t, or traverse edge (y, x) from timesteps t - 1 to t
            {
                insert2CT(x, t - 1, t);
                insert2CT(y, t, t + 1);
                insert2CT(y, x, t, t + 1);
            }
            break;
        case constraint_type::VERTEX:
            if (a == agent)
            {
                for (const auto& constraint : constraints)
                {
                    tie(a, x, y, t, type) = constraint;
                    insert2CT(x, t, t + 1);
                }
            }
            break;
        case  constraint_type::EDGE:
            assert(constraints.size() == 1);
            if (a == agent)
                insert2CT(x, y, t, t + 1);
            break;
        case constraint_type::BARRIER:
            if (a == agent)
            {
                for (auto constraint : constraints)
                {
                    tie(a, x, y, t, type) = constraint;
                    auto states = decodeBarrier(x, y, t);
                    for (const auto& state : states)
                    {
                        insert2CT(state.first, state.second, state.second + 1);
                    }

                }
            }
            break;
        case constraint_type::RANGE:
            if (a == agent)
            {
                assert(constraints.size() == 1);
                insert2CT(x, y, t + 1); // the agent cannot stay at x from timestep y to timestep t.
            }
            break;
        case constraint_type::CONSTRAINT_COUNT:
          break;
        }
}

// 每个HLNode中的constraints都只保存了新增加的，所以要不断回溯以前的constraints
// build the constraint table for the given agent at the give HL node
void ConstraintTable::insert2CT(const HLNode& node, int agent)
{
    auto curr = &node;
    while (curr->parent != nullptr)
    {
        insert2CT(curr->constraints, agent);
        curr = curr->parent;
    }
}

//求出的路径表达为约束
void ConstraintTable::insert2CT(const Path& path)
{
    int prev_location = path.front();
    int prev_timestep = 0;
    for (int timestep = 0; timestep < (int) path.size(); timestep++)
    {
        auto curr_location = path[timestep];
        if (prev_location != curr_location)
        {
            insert2CT(prev_location, prev_timestep, timestep); // add vertex conflict
            insert2CT(curr_location, prev_location, timestep, timestep + 1); // add edge conflict
            prev_location = curr_location;
            prev_timestep = timestep;
        }
    }
    insert2CT(path.back(), (int) path.size() - 1, MAX_TIMESTEP);
}

void ConstraintTable::insertLandmark(size_t loc, int t)
{
    auto it = landmarks.find(t);
    if (it == landmarks.end())
    {
        landmarks[t] = loc;
    }
    else
        assert(it->second == loc);
}

//输入只有路径
void ConstraintTable::insert2CAT(const Path& path)
{
    if (cat.empty())
    {
        cat.resize(map_size);
        cat_goals.resize(map_size, MAX_TIMESTEP);
    }
    assert(cat_goals[path.back()] == MAX_TIMESTEP);
    cat_goals[path.back()] = path.size() - 1;
    for (auto timestep = (int)path.size() - 1; timestep >= 0; timestep--)
    {
        int loc = path[timestep];
        if (cat[loc].size() <= timestep)
            //例如，timestep=1,则需要0,1时刻的点
            cat[loc].resize(timestep + 1, false);
        cat[loc][timestep] = true;
    }
    cat_max_timestep = max(cat_max_timestep, (int)path.size() - 1);
}


// build the conflict avoidance table
void ConstraintTable::insert2CAT(int agent, const vector<Path*>& paths)
{
    for (size_t ag = 0; ag < paths.size(); ++ag)
    {
        if (ag == agent || paths[ag] == nullptr)
            continue;
        insert2CAT(*paths[ag]);
    }
}

// BARRIER约束，处理矩形对称
// return the location-time pairs on the barrier in an increasing order of their timesteps
list<pair<int, int> > ConstraintTable::decodeBarrier(int x, int y, int t) const
{
    list<pair<int, int> > rst;
    int x1 = x / num_col, y1 = x % num_col;
    int x2 = y / num_col, y2 = y % num_col;
    if (x1 == x2)
    {
        if (y1 < y2)
            for (int i = min(y2 - y1, t); i>= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 - i, t - i);
            }
        else
            for (int i = min(y1 - y2, t); i >= 0; i--)
            {
                rst.emplace_back(x1 * num_col + y2 + i, t - i);
            }
    }
    else // y1== y2
    {
        if (x1 < x2)
            for (int i = min(x2 - x1, t); i>= 0; i--)
            {
                rst.emplace_back((x2 - i) * num_col + y1, t - i);
            }
        else
            for (int i = min(x1 - x2, t); i>= 0; i--)
            {
                rst.emplace_back((x2 + i) * num_col + y1, t - i);
            }
    }
    return rst;
}

// 输入的loc为点或边
bool ConstraintTable::constrained(size_t loc, int t) const
{
    assert(loc >= 0);
    // 有路标但是没占用，违反了正顶点约束。仅需要考虑顶点
    if (loc < map_size)
    {
        const auto& it = landmarks.find(t);
        if (it != landmarks.end() && it->second != loc)
            return true;  // violate the positive vertex constraint
    }

    // 点或边
    const auto& it = ct.find(loc);
    if (it == ct.end())
    {
        return false;
    }
    for (const auto& constraint: it->second)
    {
        if (constraint.first <= t && t < constraint.second)
            return true;
    }
    return false;
}

//缺一个正边约束的实现。TODO
bool ConstraintTable::constrained(size_t curr_loc, size_t next_loc, int next_t) const
{
    return constrained(getEdgeIndex(curr_loc, next_loc), next_t);
}

// 通过其他agent的路径，推算可能的冲突个数
int ConstraintTable::getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
    int rst = 0;
    if (!cat.empty())
    {
        // 点冲突
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            rst++;
        // 边冲突
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1]and cat[curr_id][next_timestep])
            rst++;
        // 目标冲突。next_timestep时刻goal已经被占用了
        if (cat_goals[next_id] < next_timestep)
            rst++;
    }
    return rst;
}

// 通过其他agent的路径，推算是否有冲突
bool ConstraintTable::hasConflictForStep(size_t curr_id, size_t next_id, int next_timestep) const
{
    if (!cat.empty())
    {
        if (cat[next_id].size() > next_timestep and cat[next_id][next_timestep])
            return true;
        if (curr_id != next_id and cat[next_id].size() >= next_timestep and cat[curr_id].size() > next_timestep and
            cat[next_id][next_timestep - 1]and cat[curr_id][next_timestep])
            return true;
        if (cat_goals[next_id] < next_timestep)
            return true;
    }
    return false;
}

bool ConstraintTable::hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const
{
    assert(curr_id != next_id);
    return !cat.empty() and curr_id != next_id and cat[next_id].size() >= next_timestep and
           cat[curr_id].size() > next_timestep and
           cat[next_id][next_timestep - 1] and cat[curr_id][next_timestep];
}

//区分collision和conflict。collision更宏观
int ConstraintTable::getFutureNumOfCollisions(int loc, int t) const
{
    int rst = 0;
    if (!cat.empty())
    {
        for (auto timestep = t + 1; timestep < cat[loc].size(); timestep++)
        {
            rst += (int)cat[loc][timestep];
        }
    }
    return rst;
}

