/*
 * solver.h
 *
 *  Created on: Aug 21, 2015
 *      Author: savio
 */

#ifndef SRC_SOLVER_H_
#define SRC_SOLVER_H_

#include <ilcplex/ilocplex.h>

#include "../include/solution.h"
#include "../include/model.h"
#include "../include/model2.h"
#include "../include/typedef.hpp"
#include "../include/UrApHMP.h"

class solver : public IloCplex {
private:
	IloNumArray2 z;
	IloNumArray2 w;
	IloNumArray3 x;
	IloNumArray3 y;
	IloNumArray4 f;

	model* mod1;
	model2* mod2;

	double obj_value;

	void run1(double tl, double UB, bool);
	void run2(double tl, double UB, bool);

public:
	solver(model*);
	solver(model2*);

	virtual ~solver();

	void run(double = MAX_DOUBLE, double = MAX_DOUBLE, bool = false);

	IloNumArray4& get_f() { return f; }
	IloNumArray3& get_x() { return x; }
	IloNumArray3& get_y() { return y; }
	IloNumArray2& get_z() { return z; }
	IloNumArray2& get_w() { return w; }
	double get_obj_value() const { return obj_value; }
};

#endif /* SRC_SOLVER_H_ */
