/*
 * ils.h
 *
 *  Created on: Apr 20, 2015
 *      Author: Sávio S. Dias
 */

#ifndef ILS_H_
#define ILS_H_

#include <utility>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <iostream>
#include "../include/FWChrono.h"
#include "../include/mt19937ar.h"
#include "../include/solution.h"
#include "../include/UrApHMP.h"
#include "../include/local_branching.h"

class ils {
private:
	// ILS Parameters
	size_t max_iterations; // Max number of iterations
	size_t max_r_iterations;
	double alpha;

	// Input instance
	uraphmp instance;

	// Current Solution
	int p; // Number of hubs to be allocated
	int r; // Number of hubs to be assigned to each node
	solution best;

	// Preprocessed mesh of probable hubs
	vector< unsigned > mesh;

	// Current Neighbors
	vector< solution > rn1;
	vector< solution > na;
	vector< solution > c2n1;

	// Nighborhood monitors
	int recent_hub;
	vector< bool > recent_assign;

	// Logs
	vector< pair< double, unsigned > > it_log;
	vector< double > times;
	FWChrono timer;

	// Private methods
	static bool my_comparison( pair< double, int >, pair< double, int > );
	void set_c2n1( const vector<solution>& );
	void set_na( const vector<solution>& );
	void set_rn1( const vector<solution>& );
	void _ils();
	void _ms_ils();
	void _lb_ils();

public:
	ils( uraphmp&, size_t, size_t, double, int, int, FWChrono& );
	ils( uraphmp&, size_t, int, int, FWChrono& );
	virtual ~ils();

	// Getters
	const solution& get_best() const;
	size_t get_max_iterations() const;
	const uraphmp& get_instance() const;
	const vector< double >& get_times() const;
	const vector< pair<double, unsigned> >& get_it_log() const;

	// Setters
	void set_best( const solution& );
	void set_instance( const uraphmp& );
	void set_max_iterations( size_t );

	// Useful Methods
	void preprocessing();

	solution constructor();
	solution greedy_randomized_constructor();

	solution local_search_rn1( solution& );
	solution local_search_c2n1( solution& );
	solution local_search_na( solution& );

	solution& r_neighborhood1( solution& );
	solution& closest2_n1( solution& );
	solution& neighborhood_a( solution& );

	// Algorithm Execution
	// TODO 4.Use a Hash Table to avoid calculus of same solutions on the neighbors **
	// 		Check whether the same hubs were allocated in a solution so to avoid the calculation
	solution& execute();

	/**
	 * @brief Run ILS with LB
	 * @details [long description]
	 * @return Best found solution
	 */
	solution& run_w_lb();
};

#endif /* ILS_H_ */
