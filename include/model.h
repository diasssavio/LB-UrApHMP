/*
 * model.h
 *
 *  Created on: Aug 21, 2015
 *      Author: savio
 */

#ifndef SRC_MODEL_H_
#define SRC_MODEL_H_

#include <sstream>
#include <ilcplex/ilocplex.h>

#include "solution.h"
#include "UrApHMP.h"
#include "typedef.hpp"

class model: public IloModel {
private:
	IloArray<IloConstraint*> fixed_consts;

	void init(); // Initialize all variables for problem
	void add_const(); // Add constraints to model
	void add_obj(); // Add objective function to model

public:
	model(IloEnv, uraphmp&, solution&);
	virtual ~model();

	// Add constraints for fixing variables
	void add_fixed_const(vector< bool >&);

	// Remove constraints for fixing variables
	void remove_fixed_const();

	// Instance & Solution
	uraphmp& instance;
	solution& sol;

	// Variables
	// traffic proportion
	IloNumVarArray4 f;

	// Assigned Hubs
	IloNumVarArray2 z;
};

#endif /* SRC_MODEL_H_ */
