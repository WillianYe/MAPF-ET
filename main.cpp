/* Copyright (C) Jiaoyang Li
* Unauthorized copying of this file, via any medium is strictly prohibited
* Confidential
* Written by Jiaoyang Li <jiaoyanl@usc.edu>, May 2020
*/


#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <climits>
#include "CBSET.h"
#include "ECBSET.h"
// #include "IPBC_CBS_ET.h"
// #include "IPBC_ET.h"

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,p", po::value<string>(), "output file for statistics")
		("outputPaths", po::value<string>(), "output file for paths")
		("agentNum,n", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0), "screen option (0: none; 1: results; 2:all)")
		("stats", po::value<bool>()->default_value(false), "write to files some detailed statistics")
		("objective,o", po::value<int>()->default_value(0), "objective option (0: maximum ET; 1: total ET)")
        ("addend,d", po::value<int>()->default_value(0), "due time addend (0,1,2)")
        ("useDtPriority", po::value<bool>()->default_value(true), "priority determined by et cost")
        ("useBuffer", po::value<bool>()->default_value(true), "use temporal buffering for IPBC-CBS-ET")

		// params for CBS node selection strategies
        ("solver", po::value<string>()->default_value("CBS-ET"), "the solver (IPBC-ET, CBS-ET, ECBS-ET, IPBC-CBS-ET)")
		("highLevelSolver", po::value<string>()->default_value("A*"), "the high-level solver for CBS (A*, A*eps, EES, NEW)")
		("inadmissibleH", po::value<string>()->default_value("Global"), "inadmissible heuristics (Zero, Global, Path, Local, Conflict)")
		("suboptimality", po::value<double>()->default_value(1), "suboptimality bound")

		// params for path coordination solvers
        ("useMDD", po::value<bool>()->default_value(false), "use MDD")
		("useStandby",po::value<bool>()->default_value(false), "use standby replanning")
		("useMultiReplan",po::value<bool>()->default_value(false), "use mutli replanning")        
		("useHeatMap",po::value<bool>()->default_value(false), "use heat map")
        ("ignoreDelay", po::value<bool>()->default_value(true), "ignore delay")

		// params for CBS improvement
		("heuristics", po::value<string>()->default_value("Zero"), "admissible heuristics for the high-level search (Zero, CG,DG, WDG)")
		("prioritizingConflicts", po::value<bool>()->default_value(true), "conflict prioirtization. If true, conflictSelection is used as a tie-breaking rule.")
		("bypass", po::value<bool>()->default_value(true), "Bypass")
		("disjointSplitting", po::value<bool>()->default_value(true), "disjoint splitting")
		("rectangleReasoning", po::value<bool>()->default_value(true), "rectangle reasoning")
		("corridorReasoning", po::value<bool>()->default_value(true), "corridor reasoning")
		("targetReasoning", po::value<bool>()->default_value(true), "target reasoning")
		// ("sipp", po::value<bool>()->default_value(false), "using SIPPS as the low-level solver")
		("restart", po::value<int>()->default_value(0), "rapid random restart times")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	if (vm["suboptimality"].as<double>() < 1)
	{
		cerr << "Suboptimal bound should be at least 1!" << endl;
		return -1;
	}

	high_level_solver_type s;
	if (vm["highLevelSolver"].as<string>() == "A*")
		s = high_level_solver_type::ASTAR;
	else if (vm["highLevelSolver"].as<string>() == "A*eps")
		s = high_level_solver_type::ASTAREPS;
	else if (vm["highLevelSolver"].as<string>() == "EES")
		s = high_level_solver_type::EES;
	else if (vm["highLevelSolver"].as<string>() == "NEW")
		s = high_level_solver_type::NEW;
	else
	{
		cout << "WRONG high level solver!" << endl;
		return -1;
	}

	if (s == high_level_solver_type::ASTAR && vm["suboptimality"].as<double>() > 1)
	{
		cerr << "A* cannot perform suboptimal search!" << endl;
		return -1;
	}

    heuristics_type h;
	if (vm["heuristics"].as<string>() == "Zero")
		h = heuristics_type::ZERO;
	else if (vm["heuristics"].as<string>() == "CG")
		h = heuristics_type::CG;
	else if (vm["heuristics"].as<string>() == "DG")
		h = heuristics_type::DG;
	else if (vm["heuristics"].as<string>() == "WDG")
		h = heuristics_type::WDG;
	else
	{
		cout << "WRONG heuristics strategy!" << endl;
		return -1;
	}

    if ((h == heuristics_type::CG || h == heuristics_type::DG) && vm["solver"].as<string>() == "ECBS-ET")
    {
        cerr << "CG or DG heuristics do not work with low level of suboptimal search!" << endl;
        return -1;
    }

	heuristics_type h_hat; // inadmissible heuristics
	if (s == high_level_solver_type::ASTAR ||
	    s == high_level_solver_type::ASTAREPS ||
	    vm["inadmissibleH"].as<string>() == "Zero")
		h_hat = heuristics_type::ZERO;
	else if (vm["inadmissibleH"].as<string>() == "Global")
		h_hat = heuristics_type::GLOBAL;
	else if (vm["inadmissibleH"].as<string>() == "Path")
		h_hat = heuristics_type::PATH;
	else if (vm["inadmissibleH"].as<string>() == "Local")
		h_hat = heuristics_type::LOCAL;
	else if (vm["inadmissibleH"].as<string>() == "Conflict")
		h_hat = heuristics_type::CONFLICT;
	else
	{
		cout << "WRONG inadmissible heuristics strategy!" << endl;
		return -1;
	}

	conflict_selection conflict = conflict_selection::EARLIEST;
	node_selection n = node_selection::NODE_CONFLICTPAIRS;

	srand((int)time(0));

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(), 
    vm["agentNum"].as<int>(), vm["addend"].as<int>());

	srand(0);
	int runs = 1 + vm["restart"].as<int>();
	//////////////////////////////////////////////////////////////////////
    // initialize the solver
	if (vm["solver"].as<string>()=="ECBS-ET")
    {
        ECBSET ecbs_et(instance, false, vm["screen"].as<int>(),vm["objective"].as<int>());
		// cout << ecbs_et.getSolverName() << endl;
        ecbs_et.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        ecbs_et.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        ecbs_et.setBypass(vm["bypass"].as<bool>());
        ecbs_et.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        ecbs_et.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        ecbs_et.setHeuristicType(h, h_hat);
        ecbs_et.setTargetReasoning(vm["targetReasoning"].as<bool>());
        ecbs_et.setMutexReasoning(false);
        ecbs_et.setConflictSelectionRule(conflict);
        ecbs_et.setNodeSelectionRule(n);
        ecbs_et.setSavingStats(vm["stats"].as<bool>());
        ecbs_et.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
		int lowerbound = 0;
		if (vm["objective"].as<int>()== 1)
        	lowerbound = - INT_MAX;
        for (int i = 0; i < runs; i++)
        {
            ecbs_et.clear();
            ecbs_et.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += ecbs_et.runtime;
            if (ecbs_et.solution_found)
                break;
            lowerbound = ecbs_et.getLowerBound();
            ecbs_et.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        ecbs_et.runtime = runtime;
        if (ecbs_et.solution_found && vm.count("output"))
            ecbs_et.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
        if (ecbs_et.solution_found && vm.count("outputPaths"))
            ecbs_et.savePaths(vm["outputPaths"].as<string>());
        /*size_t pos = vm["output"].as<string>().rfind('.');      // position of the file extension
        string output_name = vm["output"].as<string>().substr(0, pos);     // get the name without extension
        cbs.saveCT(output_name); // for debug*/
        if (vm["stats"].as<bool>())
            ecbs_et.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
        ecbs_et.clearSearchEngines();
    }
    else if(vm["solver"].as<string>()=="CBS-ET")
    {
        CBSET cbs_et(instance, false, vm["screen"].as<int>(),vm["objective"].as<int>());
        cbs_et.setPrioritizeConflicts(vm["prioritizingConflicts"].as<bool>());
        cbs_et.setDisjointSplitting(vm["disjointSplitting"].as<bool>());
        cbs_et.setBypass(vm["bypass"].as<bool>());
        cbs_et.setRectangleReasoning(vm["rectangleReasoning"].as<bool>());
        cbs_et.setCorridorReasoning(vm["corridorReasoning"].as<bool>());
        cbs_et.setHeuristicType(h, h_hat);
        cbs_et.setTargetReasoning(vm["targetReasoning"].as<bool>());
        cbs_et.setMutexReasoning(false);
        cbs_et.setConflictSelectionRule(conflict);
        cbs_et.setNodeSelectionRule(n);
        cbs_et.setSavingStats(vm["stats"].as<bool>());
        cbs_et.setHighLevelSolver(s, vm["suboptimality"].as<double>());
        //////////////////////////////////////////////////////////////////////
        // run
        double runtime = 0;
        int lowerbound = 0;
        for (int i = 0; i < runs; i++)
        {
            cbs_et.clear();
            cbs_et.solve(vm["cutoffTime"].as<double>() / runs, lowerbound);
            runtime += cbs_et.runtime;
            if (cbs_et.solution_found)
                break;
            lowerbound = cbs_et.getLowerBound();
            cbs_et.randomRoot = true;
            cout << "Failed to find solutions in Run " << i << endl;
        }
        cbs_et.runtime = runtime;
        if (cbs_et.solution_found && vm.count("output"))
            cbs_et.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
        if (cbs_et.solution_found && vm.count("outputPaths"))
            cbs_et.savePaths(vm["outputPaths"].as<string>());
        if (vm["stats"].as<bool>())
            cbs_et.saveStats(vm["output"].as<string>(), vm["agents"].as<string>());
        cbs_et.clearSearchEngines();
    }
    // else if(vm["solver"].as<string>()=="IPBC-ET"){
    //     IPBC_ET ipbc_et(instance,0,vm["screen"].as<int>(),0,0,INT_MAX,	
    //         vm["useMDD"].as<bool>(),
    //         vm["useStandby"].as<bool>(),
    //         vm["useHeatMap"].as<bool>(),
    //         vm["useMultiReplan"].as<bool>(),
    //         vm["ignoreDelay"].as<bool>(),
    //         vm["objective"].as<int>(),
    //         vm["useDtPriority"].as<bool>());
    //     ipbc_et.solve(vm["cutoffTime"].as<double>());
    //     if (vm.count("output"))
    //         ipbc_et.saveResults(vm["output"].as<string>(), vm["map"].as<string>());
    //     if (vm.count("outputPaths"))
    //         ipbc_et.savePaths(vm["outputPaths"].as<string>());
    //     if (vm["stats"].as<bool>())
    //         ipbc_et.saveStats(vm["output"].as<string>(), vm["map"].as<string>());
    // }
    // else if(vm["solver"].as<string>()=="IPBC-CBS-ET"){
    //     //IPBC-CBS-ET does not support MDD or standby replanning
    //     IPBC_CBS_ET ipbc_cbs_et(instance,0,vm["screen"].as<int>(),0,0,INT_MAX,	
    //         false,
    //         false,
    //         vm["useHeatMap"].as<bool>(),
    //         vm["useMultiReplan"].as<bool>(),
    //         vm["ignoreDelay"].as<bool>(),
    //         vm["objective"].as<int>(),
    //         vm["useDtPriority"].as<bool>(),
    //         vm["useBuffer"].as<bool>());
    //     ipbc_cbs_et.solve(vm["cutoffTime"].as<double>());
    //     ipbc_cbs_et.solveCBS(vm["prioritizingConflicts"].as<bool>(),
    //         vm["disjointSplitting"].as<bool>(),vm["bypass"].as<bool>(),
    //         vm["rectangleReasoning"].as<bool>(),vm["corridorReasoning"].as<bool>(),h,h_hat,
    //         vm["targetReasoning"].as<bool>(),conflict,n,vm["stats"].as<bool>(),s,
    //         vm["suboptimality"].as<double>(),runs);
    //     ipbc_cbs_et.verify();
    //     if (vm.count("output"))
    //         ipbc_cbs_et.saveResults(vm["output"].as<string>(), vm["map"].as<string>());
    //     if (vm.count("outputPaths"))
    //         ipbc_cbs_et.savePaths(vm["outputPaths"].as<string>());
    //     if (vm["stats"].as<bool>())
    //         ipbc_cbs_et.saveStats(vm["output"].as<string>(), vm["map"].as<string>());
    // }
    else{
		cout << "WRONG solver!" << endl;
		return -1;        
    }

	return 0;

}