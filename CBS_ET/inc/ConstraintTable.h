#pragma once
#include "common.h"
#include "CBSNode.h"


// 单agent
// 注意：conflict table和conflict的定义不一样
//constraint table, key: 顶点或边, value: 时间段向量
// constraint avoidance table，vector<vector<bool>>，所有顶点的向量，值为时间。map_size长度。由不包含当前agent的其他agent的paths构建
// landmarks:单agent，key：时间点，value：顶点。有序map，按时间顺序

class ConstraintTable
{
public:
    int length_min = 0; //最小路径时间长度约束。满足此约束的路径长度应该大于此值
    int length_max = MAX_TIMESTEP; //最大路径时间长度约束。满足此约束的路径长度应该小于此值
    size_t num_col;
    size_t map_size;

    // 最早能占用节点的时间
    int getHoldingTime(int location, int earliest_timestep) const; // the earliest timestep that the agent can hold the location after earliest_timestep
    // 永远停留在最后时刻的位置
    int getMaxTimestep() const; // everything is static after the max timestep
    // 最后发生冲突的时间
    int getLastCollisionTimestep(int location) const;
    // void clear(){ct.clear(); cat_small.clear(); cat_large.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

    bool constrained(size_t loc, int t) const;
    bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
    int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasConflictForStep(size_t curr_id, size_t next_id, int next_timestep) const;
    bool hasEdgeConflict(size_t curr_id, size_t next_id, int next_timestep) const;
    int getFutureNumOfCollisions(int loc, int t) const;

    ConstraintTable(size_t num_col, size_t map_size) : num_col(num_col), map_size(map_size) {}
    ConstraintTable(const ConstraintTable& other) { copy(other); }
    ~ConstraintTable() = default;

    void copy(const ConstraintTable& other)
    {
        length_min = other.length_min;
        length_max = other.length_max;
        num_col = other.num_col;
        map_size = other.map_size;
        ct = other.ct;
        ct_max_timestep = other.ct_max_timestep;
        cat = other.cat;
        cat_goals = other.cat_goals;
        cat_max_timestep = other.cat_max_timestep;
        landmarks = other.landmarks;
    }
    void init(const ConstraintTable& other) { copy(other); }
    void clear()
    {
        ct.clear();
        landmarks.clear();
        cat.clear();
    }
    void insert2CT(const HLNode& node, int agent); // build the constraint table for the given agent at the give node
    void insert2CT(const list<Constraint>& constraints, int agent); // insert constraints for the given agent to the constraint table
    void insert2CT(const Path& path); // insert a path to the constraint table
    void insert2CT(size_t loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
    void insert2CT(size_t from, size_t to, int t_min, int t_max); // insert an edge constraint to the constraint table
    void insert2CAT(int agent, const vector<Path*>& paths); // build the conflict avoidance table using a set of paths
    void insert2CAT(const Path& path); // insert a path to the collision avoidance table
    //int getCATMaxTimestep() const {return cat_max_timestep;}

protected:
    friend class ReservationTable;
    // 点或边-时间段
    typedef unordered_map<size_t, list< pair<int, int> > > CT; // constraint table
    CT ct; // location -> time range, or edge -> time range
    int ct_max_timestep = 0;
    // typedef unordered_map<size_t, set< pair<int, int> > > CAT; // conflict avoidance table // location -> time range, or edge -> time range
    //cat是 空间-时间二维bool矩阵。由其他所有agent给定的路径构建
    typedef vector< vector<bool> > CAT;
    CAT cat;
    int cat_max_timestep = 0;
    // 专门给目标点建立一个cat_goals。其索引为vertex_id，值为时间
    vector<int> cat_goals;

    // landmarks: 必须在某时间点上到达某位置
    map<int, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

    void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep
    list<pair<int, int> > decodeBarrier(int B1, int B2, int t) const;

    // 边的索引，最多map_size*map_size条边。最小索引map_size，最大索引：(1+map_size)*map_size+map_size
    // 因为顶点和边的索引不重复，可以放在constraint table里面
    inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }
};
