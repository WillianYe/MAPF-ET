#pragma once
#include "common.h"

// 冲突和约束。冲突生成约束。检测出来一个涉及2个agent的冲突，可为2个agent生成多个约束
// 这里的约束和constraintTable中的约束不同。后者按照agent进行组织

enum conflict_type { MUTEX, TARGET, CORRIDOR, RECTANGLE, STANDARD, TYPE_COUNT };
// 标准(顶点、边)冲突-> VERTEX约束、EDGE约束
// 走廊冲突-> RANGE约束
// 矩形冲突-> BARRIER约束
// 目标冲突-> LEQLENGTH, GLENGTH约束
// 互斥冲突-> 

enum conflict_priority { DOUBLE_CARDINAL, SINGLE_CARDINAL, LATENT_CARDINAL, CARDINAL, PSEUDO_CARDINAL, SEMI, NON, UNKNOWN, PRIORITY_COUNT };
// DOUBLE_CARDINAL, SINGLE_CARDINAL, ZERO_CARDINAL是为MAPF-DT新增的
// 冲突优先级，冲突排序第一原则。列举的顺序不能变！
// Pseudo-cardinal conflicts are semi-/non-caridnal conflicts between dependent agents. the two agents are dependent, although resolving this conflict might not increase the cost。即虽然属于semi-/non-caridnal，但agent1和agent2是相互依赖的，路径总长度会增加。
// We prioritize them over normal semi-/non-caridnal conflicts 

enum conflict_selection {RANDOM, EARLIEST, CONFLICTS, MCONSTRAINTS, FCONSTRAINTS, WIDTH, SINGLETONS};
// 冲突排序第二原则

enum constraint_type { LEQLENGTH, GLENGTH, RANGE, BARRIER, VERTEX, EDGE, POSITIVE_VERTEX, POSITIVE_EDGE, CONSTRAINT_COUNT};
// <agent, loc, -1, t, VERTEX> 顶点约束：不能在t时刻到达顶点loc
// <agent, from, to, t, EDGE> 边约束：不能在t时刻到达某一边from->to
// <agent, loc, -1, t, POSITIVE_VERTEX> 正顶点约束：必须在t时刻到达顶点loc，Disjoint splitting
// <agent, from, to, t, POSITIVE_EDGE> 正边约束：必须在t时刻到达边from->to，Disjoint splitting
// <agent, loc, -1, t, LEQLENGTH>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t. 路径长度约束：路径小于某一长度。其他agent不能位于顶点loc，用于解决target conflict
// <agent, loc, -1, t, GLENGTH>: path of agent_id should be of length at least t + 1. 路径长度约束：路径大于某一长度，用于解决target conflict
// <agent, loc, 0, t, RANGE> ：范围约束，t之前不能到达走廊范围的端点位置loc，，用于解决corridor conflict
// <agent, B1, B2, t, BARRIER> ：t之前不能进入一个矩形，，用于解决rectangle conflict

typedef std::tuple<int, int, int, int, constraint_type> Constraint;

std::ostream& operator<<(std::ostream& os, const Constraint& constraint);


class Conflict
{
public:
	int a1;
	int a2;
	list<Constraint> constraint1;
	list<Constraint> constraint2;
	conflict_type type;
	conflict_priority priority = conflict_priority::UNKNOWN;
	double secondary_priority = 0; // used as the tie-breaking criteria for conflict selection
    int getConflictId() const { return int(type); }  // int(PRIORITY_COUNT) * int(type) + int(priority); }

	//顶点冲突-> 2个顶点约束
	void vertexConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::VERTEX);
		this->constraint2.emplace_back(a2, v, -1, t, constraint_type::VERTEX);
		type = conflict_type::STANDARD;
	}
		
	//边冲突-> 2个边约束
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(a2, v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}

	//走廊冲突-> 2个范围约束
	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v1, 0, t1, constraint_type::RANGE);
		this->constraint2.emplace_back(a2, v2, 0, t2, constraint_type::RANGE);
		type = conflict_type::CORRIDOR;
	}

	//矩形冲突-> 2个约束
	bool rectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
	                         int Rg_t, const list<Constraint>& constraint1, const list<Constraint>& constraint2) // For RM
	{
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1 = constraint1;
		this->constraint2 = constraint2;
		type = conflict_type::RECTANGLE;
		return true;
	}

	//目标冲突-> 2个路径长度约束。a2要用到a1的goal
	void targetConflict(int a1, int a2, int v, int t)
	{
        constraint1.clear();
        constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		this->constraint1.emplace_back(a1, v, -1, t, constraint_type::LEQLENGTH); //a1提前到达v并永远停下，其他agent不能再进入v
		this->constraint2.emplace_back(a1, v, -1, t, constraint_type::GLENGTH); //a1在t之后到达并允许其他agent在t之前暂时到达v
		type = conflict_type::TARGET;
	}

	//互锁冲突-> 2个路径长度约束
	void mutexConflict(int a1, int a2)
	{
		constraint1.clear();
		constraint2.clear();
		this->a1 = a1;
		this->a2 = a2;
		type = conflict_type::MUTEX;
		priority = conflict_priority::CARDINAL;
		// TODO add constraints from mutex reasoning
	}

	
};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);
