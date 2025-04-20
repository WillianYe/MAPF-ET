#pragma once
#include "SingleAgentSolver.h"


// 单agent的MDD node
class MDDNode
{
public:
	int location;
	int level; // time
  	int cost; // minimum cost of path traversing this MDD node

	list<MDDNode*> children;
	list<MDDNode*> parents;

	MDDNode(int currloc, MDDNode* parent)
	{
		location = currloc; 
		if(parent == nullptr)
			level = 0;
		else
		{
			level = parent->level + 1;
			parents.push_back(parent);
		}
	}

	MDDNode(int location, int t): location(location), level(t) {}

	bool operator == (const MDDNode & node) const
	{
		return (this->location == node.location) && (this->level == node.level);
	}
};

// 单agent的MDD
class MDD
{
protected:
    const SingleAgentSolver* solver;

public:
	vector<list<MDDNode*>> levels;

	// build mdd of given levels
	bool buildMDD(const ConstraintTable& ct, int num_of_levels, const SingleAgentSolver* solver); 
	// build minimal MDD
	bool buildMDD(ConstraintTable& ct, const SingleAgentSolver* solver); 

	// bool buildMDD(const std::vector <std::list< std::pair<int, int> > >& constraints, int numOfLevels,
	// 	int start_location, const int* moves_offset, const std::vector<int>& my_heuristic, int map_size, int num_col);

	MDDNode* find(int location, int level) const;
	void deleteNode(MDDNode* node);
	void clear();
	// bool isConstrained(int curr_id, int next_id, int next_timestep, const std::vector< std::list< std::pair<int, int> > >& cons) const;

    void increaseBy(const ConstraintTable&ct, int dLevel, SingleAgentSolver* solver);
    MDDNode* goalAt(int level);
    void printNodes() const;

	MDD()= default;
	MDD(const MDD & cpy);
	~MDD();
};

std::ostream& operator<<(std::ostream& os, const MDD& mdd);


// 
class SyncMDDNode
{
public:
	int location;
	//int level;
	
	list<const MDDNode*> coexistingNodesFromOtherMdds;
	list<SyncMDDNode*> children;
	list<SyncMDDNode*> parents;
	
	SyncMDDNode(int currloc, SyncMDDNode* parent)
	{
		location = currloc;
		if (parent != nullptr)
		{
			//level = parent->level + 1;
			parents.push_back(parent);
		}
		//parent = NULL;
	}
	
	bool operator == (const SyncMDDNode & node) const
	{
		return (this->location == node.location);
	}
};


// 
class SyncMDD
{
public:
	vector<list<SyncMDDNode*>> levels;

	SyncMDDNode* find(int location, int level) const;
	void deleteNode(SyncMDDNode* node, int level);
	void clear();

	explicit SyncMDD(const MDD & cpy);
	~SyncMDD();
};


// 按照agent_id和约束来建立和索引对应的MDD
class MDDTable
{
public:
	double accumulated_runtime = 0;  // runtime of building MDDs
	uint64_t num_released_mdds = 0; // number of released MDDs ( to save memory)

	MDDTable(const vector<ConstraintTable>& initial_constraints,
						const vector<SingleAgentSolver*>& search_engines):
		initial_constraints(initial_constraints), search_engines(search_engines) {}
	
	void init(int number_of_agents)
	{
		lookupTable.resize(number_of_agents);
	}
	~MDDTable() { clear(); }

	MDD* findMDD(HLNode& node, int agent) const;
	MDD * getMDD(HLNode& node, int agent, int mdd_levels = -1);
	// void findSingletons(HLNode& node, int agent, Path& path);
	void clear();

private:
	int max_num_of_mdds = 10000; // per agent

	// template < class Key,                                // 键值对中键的类型
    //        class T,                                      // 键值对中值的类型
    //        class Hash = hash<Key>,                       // 容器内部存储键值对所用的哈希函数
    //        class Pred = equal_to<Key>,                   // 判断各个键值对键相同的规则
    //        class Alloc = allocator< pair<const Key,T> >  // 指定分配器对象的类型
    //        > class unordered_map;
	
	vector<unordered_map<ConstraintsHasher, MDD*, 
		ConstraintsHasher::Hasher, ConstraintsHasher::EqNode> >  lookupTable;

	const vector<ConstraintTable>& initial_constraints;
	const vector<SingleAgentSolver*>& search_engines;
	void releaseMDDMemory(int id);
};

unordered_map<int, MDDNode*> collectMDDlevel(MDD* mdd, int i);
