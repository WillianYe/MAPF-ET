#pragma once
#include <tuple>
#include <map>
#include <list>
#include <vector>
#include <stack>
#include <set>
#include <ctime>
#include <chrono>
#include <random>
#include <fstream>
#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <boost/heap/pairing_heap.hpp>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>

using boost::heap::pairing_heap;
using boost::heap::compare;
using boost::unordered_map;
using boost::unordered_set;
using boost::adjacency_list;
using boost::bidirectionalS;
using boost::undirectedS;
using boost::vecS;
using boost::edges;
using boost::in_edges;
using boost::out_edges;
using boost::add_edge;
using boost::remove_edge;
using boost::clear_vertex;
using std::vector;
using std::list;
using std::stack;
using std::set;
using std::map;
using std::get;
using std::tuple;
using std::make_tuple;
using std::pair;
using std::make_pair;
using std::tie;
using std::min;
using std::max;
using std::shared_ptr;
using std::make_shared;
using std::clock;
using std::cout;
using std::endl;
using std::ofstream;
using std::cerr;
using std::string;
using std::fill;
using std::abs;
using std::swap;
using std::queue;

// #define NDEBUG 

#define MAX_TIMESTEP INT_MAX / 2
#define MAX_COST INT_MAX / 2
#define MAX_NODES INT_MAX / 2

typedef vector<int> Path;
std::ostream& operator<<(std::ostream& os, const Path& path);

bool isSamePath(const Path& p1, const Path& p2);

// Only for three-tuples of std::hash-able types for simplicity.
// You can of course template this struct to allow other hash functions
/*struct three_tuple_hash {
    template <class T1, class T2, class T3>
    std::size_t operator () (const std::tuple<T1, T2, T3> &p) const {
        auto h1 = std::hash<T1>{}(get<0>(p));
        auto h2 = std::hash<T2>{}(get<1>(p));
        auto h3 = std::hash<T3>{}(get<2>(p));
        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2 ^ h3;
    }
};*/

