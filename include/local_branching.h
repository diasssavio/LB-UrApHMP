#ifndef LOCAL_BRANCHING_H_
#define LOCAL_BRANCHING_H_

#include <ilcplex/ilocplex.h>

#include "../include/model.h"
#include "../include/solver.h"
#include "../include/typedef.hpp"

/**
 * @brief Local Branching class
 * @details [long description]
 * 
 */
class local_branching {
private:
	// Input Data
	IloEnv& env; // CPLEX Environment
	solution& sol; // Reference solution for LBC
	model& mod; // Problem model

	// LBC Solution
	IloNumArray2 _z;
	IloNumArray4 _f;
	solution result;

public:
	local_branching(IloEnv&, solution&, model&);

	~local_branching();

	const IloNumArray2& get_z() const;
	const IloNumArray4& get_f() const;
	const solution& get_result() const;

	bool run(double, unsigned = 1, unsigned = 0);
	bool run(double, double, bool = false, unsigned = 1, unsigned = 0);
};

#endif /* LOCAL_BRANCHING_H_ */