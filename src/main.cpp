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
#include "../include/ils.h"

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
	IloEnv env;

	try{
		FWChrono timer;
		timer.start();
		ils ILS(instance, max_it, p, r, timer);
		solution result = ILS.run_w_lb();
		timer.stop();
	}catch(IloException& e){
		cerr << "Concert Exception" << e << endl;
	}
	// Closing the environment
	env.end();

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
