#include <cstdio>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <ilcplex/ilocplex.h>

#include "../include/FWChrono.h"
#include "../include/mt19937ar.h"
#include "../include/UrApHMP.h"
#include "../include/solution.h"
#include "../include/model.h"
#include "../include/model2.h"
#include "../include/ils.h"

#define ARGS_NUM 6

using namespace std;

template<typename T>
T string_to(const string& s){
	istringstream i(s);
	T x;
	if (!(i >> x)) return 0;
	return x;
}

template<typename T>
string to_string2(const T& t){
  stringstream ss;
  if(!(ss << t)) return "";
  return ss.str();
}

void drawing(ofstream&, double**, solution&);

ILOSTLBEGIN
int main(int argc, char* args[]){
	if(argc - 1 != ARGS_NUM)
		exit(EXIT_FAILURE);

	// Reading input file
	int p = string_to<int>(args[1]);
	int r = string_to<int>(args[2]);
	double X = string_to<double>(args[3]), alpha_1 = string_to<double>(args[4]), delta = string_to<double>(args[5]);
	
	unsigned long seed = string_to<unsigned long>(args[6]);
	init_genrand(seed);

	int n;
	scanf("%d", &n);
	
	uraphmp instance(n, X, alpha_1, delta);

	vector< vector< double> > aux;
	for(int i = 0; i < n; i++){
		vector< double > aux2;
		for(int j = 0; j < n; j++){
			double temp;
			scanf("%lf", &temp);
			aux2.push_back(temp);
		}
		aux.push_back(aux2);
	}
	instance.set_traffics(aux);

	aux.clear();
	for(int i = 0; i < n; i++){
		vector< double > aux2;
		for(int j = 0; j < n; j++){
			double temp;
			scanf("%lf", &temp);
			aux2.push_back(temp);
		}
		aux.push_back(aux2);
	}
	instance.set_distances(aux);
	aux.clear();

	unsigned max_it = 0.1 * n;
	// unsigned max_lb_it = 0;

	// Initializing cplex environment
	/*IloEnv env;
	solution initial(instance, p, r);
	unsigned hubs[] = {0, 3, 6, 11, 17};
	set< unsigned > a_hubs(hubs, hubs+5);
	initial.set_alloc_hubs(a_hubs);
	solution result;

	try{
		model mod(env, instance, initial);
		solver cplex(&mod);
		cplex.run();
		// result = solution(instance, p, r, cplex.get_z(), cplex.get_w(), cplex.get_x(), cplex.get_y(), cplex.get_obj_value());
		result = solution(instance, p, r, cplex.get_z(), cplex.get_f(), cplex.get_obj_value());
	}catch(IloException& e){
		cerr << "Concert Exception" << e << endl;
	}

	// Closing the environment
	env.end();*/

	FWChrono timer;
	timer.start();
	ils ILS(instance, max_it, p, r, timer);
	solution result = ILS.run_w_lb();
	timer.stop();

	printf("TOTAL EXECUTION TIME: %.2lf\n", timer.getStopTime());
	result.show_data();

	return 0;
}

void drawing(ofstream& out, double** z, solution& sol){
	out << "strict graph G {" << endl;
	
	// Drawing the nodes & hubs
	for(int i = 0; i < sol.get_instance().get_n(); i++){
		out << i << "[label=" << i+1 << "]";
		if(z[i][i] != 0.0)
			out << "[style=filled]";
		out << ";" << endl;
	}

	// Drawing the edges
	for(int i = 0; i < sol.get_instance().get_n(); i++)
		for(int j = 0; j < sol.get_instance().get_n(); j++){
			if(i == j) continue;
			char aux[10];
			sprintf(aux, "%.2lf", z[i][j]);
			if(z[i][j] == 0.5)
				out << i << "--" << j << "[style=dotted];" << endl;
			if((z[i][j] != 0.0) && (z[i][j] < 0.5))
				out << i << "--" << j << "[color=red, label=" << aux << "];" << endl;
			if((z[i][j] > 0.5) && (z[i][j] != 1.0))
				out << i << "--" << j << "[color=blue, label=" << aux << "];" << endl;
			if(z[i][j] > 0.5)
				out << i << "--" << j << ";" << endl;
		}

	out << "}";
}
