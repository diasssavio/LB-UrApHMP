/*
 * ils.cpp
 *
 *  Created on: Apr 20, 2015
 *      Author: Sávio S. Dias
 */

#include "../include/ils.h"

ils::ils( uraphmp& instance, size_t max_iterations, size_t _max_r, double _alpha, int p, int r, FWChrono& timer ) : max_iterations(max_iterations), p(p), r(r), alpha(_alpha), max_r_iterations(_max_r) {
	this->set_instance(instance);
	this->mesh = vector< unsigned >(instance.get_n());
	for(unsigned i = 0; i < mesh.size(); i++)
		this->mesh[i] = i;
	this->timer = timer;
	this->recent_hub = -1;
	this->recent_assign = std::vector< bool >(instance.get_n());
}

ils::ils( uraphmp& instance, size_t max_iterations, int p, int r, FWChrono& timer ) : max_iterations(max_iterations), p(p), r(r) {
	this->set_instance(instance);
	this->mesh = vector< unsigned >(instance.get_n());
	for(unsigned i = 0; i < mesh.size(); i++)
		this->mesh[i] = i;
	this->timer = timer;
	this->alpha = 0.0;
	this->max_r_iterations = 0;
	this->recent_hub = -1;
	this->recent_assign = std::vector< bool >(instance.get_n());
}

ils::~ils() { }

void ils::set_c2n1(const vector<solution>& c2n1) { this->c2n1 = c2n1; }

void ils::set_na(const vector<solution>& na) { this->na = na; }

void ils::set_rn1(const vector<solution>& rn1) { this->rn1 = rn1; }

const solution& ils::get_best() const { return best; }

const uraphmp& ils::get_instance() const { return instance; }

size_t ils::get_max_iterations() const { return max_iterations; }

const vector<pair<double, unsigned> >& ils::get_it_log() const { return it_log; }

const vector<double>& ils::get_times() const { return times; }

void ils::set_best(const solution& best) { this->best = best; }

void ils::set_instance(const uraphmp& instance) { this->instance = instance; }

void ils::set_max_iterations(size_t maxIterations) { max_iterations = maxIterations; }

bool ils::my_comparison( pair< double, int > p1, pair< double, int > p2 ){ return (p1.first < p2.first); }

void ils::preprocessing(){
	vector< vector< double > > traffics = instance.get_traffics();
	vector< vector< double > > distances = instance.get_distances();
	int n = instance.get_n();

	vector< pair< double, unsigned > > aux;
	for(int h = 0; h < n; h++){
		// Creating the e(j,h)
		vector< double > e;
		for(int j = 0; j < n; j++){
			if(h == j) continue;

			double aux = accumulate(traffics[j].begin(), traffics[j].end(), 0.0);
			aux *= distances[j][h];
			e.push_back(aux);
		}

		// Sorting the elements of e(j,h)
		sort(e.begin(), e.end());

		// Chosing the k = |n/p| elements to compose g(h)
		int k = n/p;
		double sum = accumulate(e.begin(), e.begin() + k, 0.0);
		aux.push_back(make_pair(sum, h));
	}

	// Eliminating the x percent most expensive
	int x = (0.1 * n);
	sort(aux.begin(), aux.end(), my_comparison);
	vector< unsigned > mesh;
	for(unsigned i = 0; i < n - x; i++)
		mesh.push_back(aux[i].second);

	this->mesh = mesh;
}

solution ils::constructor(){
	vector< vector< double > > traffics = instance.get_traffics();
	vector< vector< double > > distances = instance.get_distances();

	solution sol(instance, p, r);

	// Step 1 - Locating the hubs - alloc_hubs
	set< unsigned > hubs;
	for(int p = 0; p < sol.get_p(); p++){
		// Calculating the g(h) for each node unselected as hub - The Candidate List
		vector< pair< double, unsigned > > g;
		for(unsigned h = 0; h < mesh.size(); h++){
			// Adaptive part of the procedure
			if(hubs.find(mesh[h]) != hubs.end()) continue;

			// Creating the e(j,h)
			vector< double > e;
			for(int j = 0; j < instance.get_n(); j++){
				// Adaptive part of the procedure
				if(hubs.find(j) != hubs.end()) continue;

				double aux = accumulate(traffics[j].begin(), traffics[j].end(), 0.0);
				aux *= distances[j][ mesh[h] ];
				e.push_back(aux);
			}

			// Sorting the elements of e(j,h)
			sort(e.begin(), e.end());

			// Choosing the k = |n/p| elements to compose g(h)
			int k = instance.get_n() / sol.get_p();
			double sum = accumulate(e.begin(), e.begin() + k, 0.0);
			g.push_back(make_pair(sum, mesh[h]));
		}

		pair< double, int > g_min = *min_element(g.begin(), g.end(), my_comparison);
		hubs.insert(g_min.second);
	}
	sol.set_alloc_hubs(hubs);

	// Step 2 - Assignment of the r hubs - assigned_hubs
	sol.assign_hubs();

	// Step 3 - Route traffics
	sol.route_traffics();

	return sol;
}

solution ils::greedy_randomized_constructor(){
	vector< vector< double > > traffics = instance.get_traffics();
	vector< vector< double > > distances = instance.get_distances();

	solution sol(instance, p, r);

	// Step 1 - Locating the hubs - alloc_hubs
	set< unsigned > hubs;
	for(int p = 0; p < sol.get_p(); p++){
		// Calculating the g(h) for each node unselected as hub - The Candidate List
		vector< pair< double, unsigned > > g;
		for(unsigned h = 0; h < mesh.size(); h++){
			// Adaptive part of the procedure
			if(hubs.find(mesh[h]) != hubs.end()) continue;

			// Creating the e(j,h)
			vector< double > e;
			for(int j = 0; j < instance.get_n(); j++){
				// Adaptive part of the procedure
				if(hubs.find(j) != hubs.end()) continue;

				double aux = accumulate(traffics[j].begin(), traffics[j].end(), 0.0);
				aux *= distances[j][ mesh[h] ];
				e.push_back(aux);
			}

			// Sorting the elements of e(j,h)
			sort(e.begin(), e.end());

			// Choosing the k = |n/p| elements to compose g(h)
			int k = instance.get_n()/sol.get_p();
			double sum = accumulate(e.begin(), e.begin() + k, 0.0);
			g.push_back(make_pair(sum, mesh[h]));
		}

		// Creating the RCL based on g(h) -- GRASP
		double g_min = (*min_element(g.begin(), g.end(), my_comparison)).first;
		double g_max = (*max_element(g.begin(), g.end(), my_comparison)).first;
		vector< unsigned > RCL;
		for(unsigned i = 0; i < g.size(); i++)
			if(g[i].first <= (g_min + this->alpha*(g_max - g_min)))
				RCL.push_back(g[i].second);

		// Selecting randomly the hub from RCL
		hubs.insert(RCL[ genrand_int32() % RCL.size() ]);

//		pair< double, int > g_min = *min_element(g.begin(), g.end(), my_comparison);
//		hubs.push_back(g_min.second);
	}
	sol.set_alloc_hubs(hubs);

	// Step 2 - Assignment of the r hubs - assigned_hubs
	sol.assign_hubs();

	// Step 3 - Route traffics
	sol.route_traffics();

	return sol;
}

solution ils::local_search_rn1( solution& p_sol ){
	// Checking the best elements in the Neighborhood 1
	solution partial = p_sol;
	solution improved;
	bool is_improved = true;
	while(is_improved){
		improved = r_neighborhood1(partial);
		// improved = neighbors[ neighbors.size() - 1 ];
		if(improved.get_total_cost() < partial.get_total_cost()){
			// Defining the recent found hub & restarting the recent assignments
			std::vector< unsigned > v(1);
			std::vector< unsigned >::iterator it = set_difference(improved.get_alloc_hubs().begin(), improved.get_alloc_hubs().end(), partial.get_alloc_hubs().begin(), partial.get_alloc_hubs().end(), v.begin());
			recent_hub = v[0];
			fill_n(recent_assign.begin(), instance.get_n(), false);

			partial = improved;
		}
		else is_improved = false;
	}

	return partial;
}

solution ils::local_search_c2n1( solution& p_sol ){
	// TODO 1.2.Test the use w/ RN1

	// Checking the best elements in the Adjacent points
	solution partial = p_sol;
	solution improved;
	bool is_improved = true;
	while(is_improved){
		improved = closest2_n1(partial);
		// improved = neighbors[ neighbors.size() - 1 ];
		if(improved.get_total_cost() < partial.get_total_cost())
			partial = improved;
		else is_improved = false;
	}

	return partial;
}

solution ils::local_search_na( solution& p_sol ){
	// Checking the best solution in the Neighborhood A

	solution partial = p_sol;
	solution improved;
	bool is_improved = true;
	while(is_improved){
		improved = neighborhood_a(partial);
		// improved = neighbors[ neighbors.size() - 1 ];
		if(improved.get_total_cost() < partial.get_total_cost())
			partial = improved;
		else is_improved = false;
	}

	return improved;
}

solution& ils::r_neighborhood1( solution& p_sol ){
	/**
	 * Generate a Neighborhood based on a one-point exchange of a random Hub
	 * in p_sol (partial solution) given.
	 */

	// Generating Neighborhood
	vector< solution > neighbors;
	for(unsigned i = 0; i < mesh.size(); i++){
		if(p_sol.is_hub(mesh[i])) continue;

		unsigned h;
		set< unsigned > hubs(p_sol.get_alloc_hubs());
		set< unsigned >::iterator it;

		// Chosing a hub that wasn't recently inserted
		bool can_change = false;
		while(!can_change){
			it = hubs.begin();
			h = genrand_int32() % p; // hub to be exchanged
			advance(it, h);
			if(*it != recent_hub) can_change = true;				
		}

		hubs.erase(it);
		hubs.insert(mesh[i]);
		// hubs[h] = mesh[i];

		solution s1(instance, p, r);
		s1.set_alloc_hubs(hubs);
		s1.assign_hubs();
		s1.route_traffics();
		neighbors.push_back(s1);
		// if(s1.get_cost() < p_sol.get_cost()) break;
		// 	set_rn1(neighbors);
		// 	return rn1[ rn1.size() - 1 ];
	}
	set_rn1(neighbors);

	return *min_element(rn1.begin(), rn1.end(), solution::my_sol_comparison);
	// return rn1[ rn1.size() - 1 ];
}

solution& ils::closest2_n1( solution& p_sol ){
	/**
	 * Generate a Neighborhood based on a one-point exchange of the two closest points of Hubs
	 * in p_sol (partial solution) given.
	 */
	// TODO 1.1.Check the generation of previously calculated solutions as neighbors
	//		There's many intersection in this neighborhood

	vector< vector< double > > distances = instance.get_distances();

	// Generating Neighborhood
	vector< solution > neighbors; // Analyzing the 2 nearest points for each hub
	set< unsigned > alloc_hubs = p_sol.get_alloc_hubs();
	set< unsigned >::iterator i;
	unsigned count = 0;
	for(i = alloc_hubs.begin(); i != alloc_hubs.end(); i++, count++){
		// Finding the first two non-hub nodes
		unsigned j = 0;
		while(p_sol.is_hub(mesh[j])) j++;
		unsigned k = j + 1;
		while(p_sol.is_hub(mesh[k])) k++;
		pair< unsigned, double > min1, min2;
		if(distances[*i][mesh[k]] < distances[*i][mesh[j]]){
			min1 = make_pair(mesh[k], distances[*i][mesh[k]]);
			min2 = make_pair(mesh[j], distances[*i][mesh[j]]);
		} else {
			min2 = make_pair(mesh[k], distances[*i][mesh[k]]);
			min1 = make_pair(mesh[j], distances[*i][mesh[j]]);
		}
		// Getting the two Euclidian closest non-hub nodes
		for(unsigned j = k + 1; j < mesh.size(); j++){
			if(*i == mesh[j] || p_sol.is_hub(mesh[j])) continue; // main diagonal & hub nodes avoidance
			if(distances[*i][mesh[j]] < min1.second){
				min2 = min1;
				min1 = make_pair(mesh[j], distances[*i][mesh[j]]);
			}
		}

		// Making the solution objects
		set< unsigned > hubs1(alloc_hubs);
		set< unsigned > hubs2(alloc_hubs);
		set< unsigned >::iterator it = hubs1.begin();
		advance(it, count);
		hubs1.erase(it);
		hubs1.insert(min1.first);
		it = hubs2.begin();
		advance(it, count);
		hubs2.erase(it);
		hubs2.insert(min2.first);

		solution s1(instance, p, r);
		solution s2(instance, p, r);
		s1.set_alloc_hubs(hubs1);
		s1.assign_hubs();
		s1.route_traffics();
		s2.set_alloc_hubs(hubs2);
		s2.assign_hubs();
		s2.route_traffics();

		neighbors.push_back(s1);
		neighbors.push_back(s2);
	}

	set_c2n1(neighbors);

	return *min_element(c2n1.begin(), c2n1.end(), solution::my_sol_comparison);
}

solution& ils::neighborhood_a( solution& p_sol ){
	// Generating Neighborhood A

	vector< solution > neighbors;
	set< unsigned > hubs(p_sol.get_alloc_hubs());
	for(int i = 0; i < instance.get_n(); i++){
		// Avoiding the calculation for previous considered nodes
		if(recent_assign[i]) continue;

		vector< unsigned > assigned_hubs(p_sol.get_assigned_hubs(i));
		sort(assigned_hubs.begin(), assigned_hubs.end());

		// Finding the symmetric difference
		vector< unsigned > to_assign(p - r);
		vector< unsigned >::iterator it = set_symmetric_difference(hubs.begin(), hubs.end(), assigned_hubs.begin(), assigned_hubs.end(), to_assign.begin());
		// for(set< unsigned >::iterator j = hubs.begin(); j != hubs.end(); j++)
		// 	if(find(assigned_hubs.begin(), assigned_hubs.end(), *j) == assigned_hubs.end())
		// 		to_assign.push_back(*j);

		// Generating the neighbors
		for(int j = 0; j < this->r; j++)
			for(unsigned k = 0; k < to_assign.size(); k++){
				solution s1(instance, p, r);
				s1.set_alloc_hubs(hubs);
				s1.set_assigned_hubs(p_sol.get_assigned_hubs());
				s1.set_assigned_hub(i, j, to_assign[k]);
				s1.set_cost(p_sol.get_cost());
				s1.set_f_chosen(p_sol.get_f_chosen());
				s1.set_s_chosen(p_sol.get_s_chosen());
				s1.route_partial_traffics(i);
				neighbors.push_back(s1);
				if(s1.get_cost() < p_sol.get_cost()){
					set_na(neighbors);
					return na[ na.size() - 1 ];
				}
			}
		recent_assign[i] = true;
	}

	set_na(neighbors);

	return *min_element(na.begin(), na.end(), solution::my_sol_comparison);
}

void ils::_ils(){
	// Constructing initial solution
	solution initial = constructor();
	solution improved = initial;

	unsigned i = 1, k = 0;
	bool first = true;
	while(i < max_iterations){
		// Local Search
//		improved = local_search_c2n1(improved);
		improved = local_search_rn1(improved);

		// TODO 3.Evaluation of more than just one solution in the local search
		// Acceptance criterion & VND
		if(!first){
			if(improved.get_total_cost() < best.get_total_cost()) {
				set_best(improved);
				i = 1;
			} else { // Testing the LS_a only when LS_h doesn't improve the solution
				improved = local_search_na(improved);
				if(improved.get_total_cost() < best.get_total_cost()){
					set_best(improved);
					i = 1;
				}else i++;
			}
		} else {
			best = improved;
			first = false;
		}

		// Shaking phase
		improved = rn1[ genrand_int32() % rn1.size() ];

		// TODO 2.3.Check the cost of logs storage
		// Saving the execution logs
		it_log.push_back(make_pair(best.get_total_cost(), k++));
		times.push_back(((double) timer.getMilliSpan() / 1000));
	}
}

void ils::_ms_ils(){
	unsigned j = 1, l = 0;
	bool _first = true;

	// TODO 2.1.Update the local best or the global one
	// TODO 2.2.Fixed iterations or without improvement
	// Multi-Start iterations
	while(j < max_r_iterations){
		// Constructing initial solution
		solution initial = greedy_randomized_constructor();
		solution improved = initial;
		solution _best;

		unsigned i = 1, k = 0;
		bool first = true;
		while(i < max_iterations){
			// Local Search
			// improved = local_search_c2n1(improved);
			improved = local_search_rn1(improved);

			// Acceptance criterion & VND
			if(!first){
				if(improved.get_total_cost() < _best.get_total_cost()) {
					_best = improved;
					// set_best(improved);
					i = 1;
				} else { // Testing the LS_a only when LS_h doesn't improve the solution
					improved = local_search_na(improved);
					if(improved.get_total_cost() < _best.get_total_cost()){
						_best = improved;
						// set_best(improved);
						i = 1;
					}else i++;
				}
			} else {
				_best = improved;
				first = false;
			}

			// Shaking phase
			improved = rn1[ genrand_int32() % rn1.size() ];

			// Saving the execution logs
			it_log.push_back(make_pair(_best.get_total_cost(), k++));
			times.push_back(((double) timer.getMilliSpan() / 1000));
		}

		// MS Acceptance criterion
		if(_first){
			best = _best;
			_first = false;
		} else if(_best.get_total_cost() < best.get_total_cost()) {
			best = _best;
			j = 1;
		} else j++;

		// Saving the MS execution logs
		it_log.push_back(make_pair(best.get_total_cost(), l++));
		times.push_back(((double) timer.getMilliSpan() / 1000));
	}
}

solution& ils::execute(){
	// Pre-processing
	preprocessing();

	// Processing
	if( alpha == 0.0 ) _ils();
	else _ms_ils();

	// Post-processing
//	set_best(local_search_rn1(best));

	return best;
}

solution ils::run_w_lb(){
	// TODO Try initial reference sol with ILS & constructor heuristic
	solution initial = constructor();
	solution improved = initial;

	// Call local branching loop
	int k_min = 0;
	int k_max = 2;
	double ntl = 120;

	// Initializing CPLEX Environment
	IloEnv env;
	// int k_cur_max = k_min + 1;
	// int k_cur_min = k_min;
	
	try {
		model mod(env, instance, initial);
		local_branching lb(env, improved, mod);

		bool stopping_criterion = false;
		bool first = true;
		while(!stopping_criterion) {
			if(first){
				best = improved;
				first = false;
			}

			best.show_data();

			if(lb.run(ntl, improved.get_total_cost(), k_max, k_min)){
				improved = lb.get_result();
				if(best.get_total_cost() > improved.get_total_cost()){
					best = improved;
					// k_cur_max = k_min + 1;
					// k_cur_min = k_min;
				// } else if(k_cur_max + 1 <= k_max) {
					// k_cur_max++;
					// k_cur_min = k_cur_max;
				} else // Shaking phase
					stopping_criterion = true;
			} else
				stopping_criterion = true;
		}
	} catch(IloException& e) {
		cerr << "Concert Exception:\n" << e << endl;
	}
	// Closing the CPLEX Environment
	env.end();

	return best;
}
