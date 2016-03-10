#ifndef LOCAL_BRANCHING_H_
#define LOCAL_BRANCHING_H_

#include <ilcplex/ilocplex.h>

#include "../include/model.h"
#include "../include/model2.h"
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
	model* mod; // Problem model
	model2* mod2; // Problem model 2

	// LBC Solution
	IloNumArray2 _z;
	IloNumArray4 _f;
	IloNumArray2 _w;
	IloNumArray3 _x;
	IloNumArray3 _y;
	solution result;

	bool run1( double, double, int = 1, int = 0, bool = false );
	bool run1( double, double, int = 1, int = 0, int = 1, bool = false );
	bool run2( double, double, int = 1, int = 0, bool = false );
	bool run2( double, double, int = 1, int = 0, int = 1, bool = false );

	void set_cplex( IloCplex&, double, double, bool );

	void extract_model1( IloCplex& );
	void extract_model2( IloCplex& );

public:
	local_branching( IloEnv&, solution&, model* );

	local_branching( IloEnv&, solution&, model2* );

	~local_branching();

	const IloNumArray2& get_z() const;
	const IloNumArray4& get_f() const;
	const solution& get_result() const;

	bool run( double, double, int = 1, int = 0, bool = false );
	bool run( double, double, int = 1, int = 0, int = 1, bool = false );
};

#endif /* LOCAL_BRANCHING_H_ */