#include "../include/local_branching.h"

/**
 * @brief Local Branching class constructor
 * @details [long description]
 * 
 * @param _env CPLEX Environment
 * @param _sol Reference solution for LBC
 * @param _mod Problem model
 */
local_branching::local_branching(IloEnv& _env, solution& _sol, model* _mod) : env(_env), sol(_sol), mod(_mod) {
	_z = IloNumArray2(env);
	_f = IloNumArray4(env);
	mod2 = NULL;
}

/**
 * @brief Local Branching class constructor
 * @details [long description]
 * 
 * @param _env CPLEX Environment
 * @param _sol Reference solution for LBC
 * @param _mod Problem model 2
 */
local_branching::local_branching(IloEnv& _env, solution& _sol, model2* _mod) : env(_env), sol(_sol), mod2(_mod) {
	_z = IloNumArray2(env);
	_w = IloNumArray2(env);
	_x = IloNumArray3(env);
	_y = IloNumArray3(env);
	mod = NULL;
}

local_branching::~local_branching() { }

/**
 * @brief [brief description]
 * @details [long description]
 * @return [description]
 */
const IloNumArray2& local_branching::get_z() const { return this->_z; }

/**
 * @brief [brief description]
 * @details [long description]
 * @return [description]
 */
const IloNumArray4& local_branching::get_f() const { return this->_f; }

/**
 * @brief [brief description]
 * @details [long description]
 * @return [description]
 */
const solution& local_branching::get_result() const { return this->result; }

/**
 * @brief Run method for LB procedure
 * @details [long description]
 * 
 * @param node_time_limit Node (subproblem solution) time limit
 * @param k_max Maximum rhs of LBC expressions
 * @param k_min Minimum rhs of LBC expressions
 * @return true if found a feasible solution, false otherwise
 */
// bool local_branching::run(double ntl, unsigned k_max = 1, unsigned k_min = 0) {
bool local_branching::run(double ntl, double UB, int k_max, int k_min, bool first) {
	if(mod != NULL) return run1(ntl, UB, k_max, k_min, first);
	return run2(ntl, UB, k_max, k_min, first);
}

/**
 * @brief Run method for LB procedure with model 1
 * @details [long description]
 * 
 * @param ntl Node (subproblem solution) time limit
 * @param UB Upper Bound for solution comparison
 * @param k_max Maximum rhs of LBC expressions
 * @param k_min Minimum rhs of LBC expressions
 * @param first Stop at first better than UB solution or not
 * @return true if found a feasible solution, false otherwise
 */
// bool run(double ntl, double UB, unsigned k_max = 1, unsigned k_min = 0, bool first = false) {
bool local_branching::run1(double ntl, double UB, int k_max, int k_min, bool first) {
	int n = sol.get_instance().get_n();
	set< unsigned > alloc_hubs = sol.get_alloc_hubs();

	// Add LBC based on reference solution
	IloExpr expr1(env);
	// IloExpr expr2(env);
	for(IloInt k = 0; k < n; k++)
		if(sol.is_hub(k))
			expr1 += (1 - mod->z[k][k]);
		// else
			// expr2 += mod.z[k][k];

	// IloConstraint lbc_max = (expr1 + expr2 <= k_max);
	IloConstraint lbc_max = (expr1 <= k_max);
	stringstream lbc_name;
	lbc_name << "LBC_max";
	lbc_max.setName(lbc_name.str().c_str());
	mod->add(lbc_max);

	IloConstraint lbc_min;
	if(k_min != 0) {
		// lbc_min = (expr1 + expr2 >= _k_min);
		lbc_min = (expr1 >= k_min);
		stringstream lbc_name;
		lbc_name << "LBC_min";
		lbc_min.setName(lbc_name.str().c_str());
		mod->add(lbc_min);
	}

	// Run CPLEX on the current model
	// TODO set the stop at first better solution than a UB
	solver cplex(mod);
	cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), cplex.get_z(), cplex.get_f(), cplex.get_obj_value());
		to_ret = true;
	}
	if(cplex.getStatus() == IloAlgorithm::Status::InfeasibleOrUnbounded || cplex.getStatus() == IloAlgorithm::Status::Unknown){
		result = sol;
		to_ret = false;
	}

	// Remove LBC
	mod->remove(lbc_max);
	if(k_min != 0)
		mod->remove(lbc_min);

	return to_ret;
}

/**
 * @brief Run method for LB procedure with model 2
 * @details [long description]
 * 
 * @param ntl Node (subproblem solution) time limit
 * @param UB Upper Bound for solution comparison
 * @param k_max Maximum rhs of LBC expressions
 * @param k_min Minimum rhs of LBC expressions
 * @param first Stop at first better than UB solution or not
 * @return true if found a feasible solution, false otherwise
 */
bool local_branching::run2(double ntl, double UB, int k_max, int k_min, bool first) {
	int n = sol.get_instance().get_n();
	set< unsigned > alloc_hubs = sol.get_alloc_hubs();

	// Add LBC based on reference solution
	IloExpr expr1(env);
	// IloExpr expr2(env);
	for(IloInt k = 0; k < n; k++)
		if(sol.is_hub(k))
			expr1 += (1 - mod2->z[k][k]);
		// else
			// expr2 += mod.z[k][k];

	// IloConstraint lbc_max = (expr1 + expr2 <= k_max);
	IloConstraint lbc_max = (expr1 <= k_max);
	stringstream lbc_name;
	lbc_name << "LBC_max";
	lbc_max.setName(lbc_name.str().c_str());
	mod2->add(lbc_max);

	IloConstraint lbc_min;
	if(k_min != 0) {
		// lbc_min = (expr1 + expr2 >= _k_min);
		lbc_min = (expr1 >= k_min);
		stringstream lbc_name;
		lbc_name << "LBC_min";
		lbc_min.setName(lbc_name.str().c_str());
		mod2->add(lbc_min);
	}

	// Run CPLEX on the current model
	// TODO set the stop at first better solution than a UB
	solver cplex(mod2);
	cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), cplex.get_z(), cplex.get_w(), cplex.get_x(), cplex.get_y(), cplex.get_obj_value());
		to_ret = true;
	}
	if(cplex.getStatus() == IloAlgorithm::Status::InfeasibleOrUnbounded || cplex.getStatus() == IloAlgorithm::Status::Unknown){
		result = sol;
		to_ret = false;
	}

	// Remove LBC
	mod2->remove(lbc_max);
	if(k_min != 0)
		mod2->remove(lbc_min);

	return to_ret;
}