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
	_w = IloNumArray2(env);
	_x = IloNumArray3(env);
	_y = IloNumArray3(env);
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
	_f = IloNumArray4(env);

	// For use in hybridization
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
 * @param ntl Node (subproblem solution) time limit
 * @param UB Upper Bound
 * @param k_max Maximum rhs of LBC expressions
 * @param k_min Minimum rhs of LBC expressions
 * @param first Whether stop at first feasible sol
 * @return true if found a feasible solution, false otherwise
 */
bool local_branching::run(double ntl, double UB, int k_max, int k_min, bool first) {
	if(mod2 == NULL) return run1(ntl, UB, k_max, k_min, first);
	return run2(ntl, UB, k_max, k_min, first);
}

/**
 * @brief Run method for LB procedure w/ LBC for z_ik variables
 * @details [long description]
 * 
 * @param ntl Node (subproblem solution) time limit
 * @param UB Upper Bound
 * @param k_max Maximum rhs of LBC expressions for z_kk
 * @param k_min Minimum rhs of LBC expressions for z_kk
 * @param k2 Maximum rhs of LBC for z_ik
 * @param first Whether stop at first feasible sol
 * @return true if found a feasible solution, false otherwise
 */
bool local_branching::run(double ntl, double UB, int k_max, int k_min, int k2, bool first){
	if(mod2 == NULL) return run1(ntl, UB, k_max, k_min, k2, first);
	return run2(ntl, UB, k_max, k_min, k2, first);
}

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
	IloCplex cplex(*mod);
	set_cplex(cplex, ntl, UB, first);
	cplex.solve();

	// solver cplex(mod);
	// cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		extract_model1(cplex);
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), _z, _f, cplex.getObjValue());
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

bool local_branching::run1(double ntl, double UB, int k_max, int k_min, int k2, bool first) {
	int n = sol.get_instance().get_n();

	// Add LBC based on reference solution
	IloExpr expr1(env);
	for(IloInt k = 0; k < n; k++)
		if(sol.is_hub(k))
			expr1 += (1 - mod->z[k][k]);

	IloConstraint lbc_max = (expr1 <= k_max);
	stringstream lbc_name;
	lbc_name << "LBC_max";
	lbc_max.setName(lbc_name.str().c_str());
	mod->add(lbc_max);

	IloConstraint lbc_min;
	if(k_min != 0) {
		lbc_min = (expr1 >= k_min);
		stringstream lbc_name;
		lbc_name << "LBC_min";
		lbc_min.setName(lbc_name.str().c_str());
		mod->add(lbc_min);
	}

	// Add LBC for z_ik variables -- version 1
	// if(k2 > sol.get_r()) k2 = sol.get_r();

	// vector< vector< unsigned > > assigned = sol.get_assigned_hubs();
	// IloArray<IloConstraint> const_array(env);
	// for(IloInt i = 0; i < n; i++){
	// 	IloExpr expr(env);
	// 	IloExpr expr2(env);
	// 	for(IloInt k = 0; k < n; k++)
	// 		if(sol.is_assigned(i, k))
	// 			expr += (1 - mod->z[i][k]);
	// 		else
	// 			expr2 += mod->z[i][k];

	// 	IloConstraint lbc_max = (expr + expr2 <= k2);
	// 	stringstream lbc_name;
	// 	lbc_name << "LBC2(" << i << ")";
	// 	lbc_max.setName(lbc_name.str().c_str());
	// 	mod->add(lbc_max);

	// 	const_array.add(lbc_max);
	// }

	// Add LBC for z_ik variables -- version 2
	IloExpr aux(env);
	for(IloInt i = 0; i < n; i++){
		IloExpr expr(env);
		IloExpr expr2(env);
		for(IloInt k = 0; k < n; k++)
			if(sol.is_assigned(i, k))
				expr += (1 - mod->z[i][k]);
			else
				expr2 += mod->z[i][k];

		aux += expr + expr2;
	}

	IloConstraint lbc2 = (aux <= k2);
	stringstream lbc2_name;
	lbc2_name << "LBC2";
	lbc2.setName(lbc2_name.str().c_str());
	mod->add(lbc2);

	// Run CPLEX on the current model
	IloCplex cplex(*mod);
	set_cplex(cplex, ntl, UB, first);
	cplex.solve();

	// solver cplex(mod);
	// cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		extract_model1(cplex);
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), _z, _f, cplex.getObjValue());
		to_ret = true;
	}
	if(cplex.getStatus() == IloAlgorithm::Status::InfeasibleOrUnbounded || cplex.getStatus() == IloAlgorithm::Status::Unknown){
		result = sol;
		to_ret = false;
	}

	// Remove LBC's
	mod->remove(lbc_max);
	mod->remove(lbc2);
	// for(IloInt i = 0; i < const_array.getSize(); i++)
	// 	mod->remove(const_array[i]);
	if(k_min != 0)
		mod->remove(lbc_min);

	return to_ret;
}

/**
 * @brief Extracting variables from model 1 after solving
 * @details [long description]
 * 
 * @param cplex IloCplex solver for model 1
 */
void local_branching::extract_model1(IloCplex& cplex){
	int n = sol.get_instance().get_n();
	for(IloInt i = 0; i < n; i++){
		IloNumArray aux1(env);
		IloNumArray3 aux2(env);
		for(IloInt j = 0; j < n; j++){
			aux1.add(cplex.getValue(mod->z[i][j]));
			IloNumArray2 aux3(env);
			for(IloInt k = 0; k < n; k++){
				IloNumArray aux4(env);
				for(IloInt l = 0; l < n; l++)
					aux4.add(cplex.getValue(mod->f[i][j][k][l]));
				aux3.add(aux4);
			}
			aux2.add(aux3);
		}
		_z.add(aux1);
		_f.add(aux2);
	}
}

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
	IloCplex cplex(*mod2);
	set_cplex(cplex, ntl, UB, first);
	cplex.solve();

	// solver cplex(mod2);
	// cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		extract_model2(cplex);
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), _z, _w, _x, _y, cplex.getObjValue());

		// Refining solution from model 2 using model 1 w/ fixed variables
		/*if((cplex.getStatus() == IloAlgorithm::Status::Feasible) && (result.get_total_cost() < UB)){
			mod = new model(env, sol.get_instance(), sol);
			vector< bool > bin_alloc_hubs = result.get_bin_alloc_hubs();
			mod->add_fixed_const(bin_alloc_hubs);

			cplex.clear();
			cplex.extract(*mod);
			set_cplex(cplex, ntl, result.get_total_cost(), false);
			cplex.solve();

			if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
				extract_model1(cplex);
				result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), _z, _f, cplex.getObjValue());
			}

			// mod->remove_fixed_const();
			mod = NULL;
		}*/

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

bool local_branching::run2(double ntl, double UB, int k_max, int k_min, int k2, bool first) {
	int n = sol.get_instance().get_n();

	// Add LBC based on reference solution
	IloExpr expr1(env);
	for(IloInt k = 0; k < n; k++)
		if(sol.is_hub(k))
			expr1 += (1 - mod2->z[k][k]);

	IloConstraint lbc_max = (expr1 <= k_max);
	stringstream lbc_name;
	lbc_name << "LBC_max";
	lbc_max.setName(lbc_name.str().c_str());
	mod2->add(lbc_max);

	IloConstraint lbc_min;
	if(k_min != 0) {
		lbc_min = (expr1 >= k_min);
		stringstream lbc_name;
		lbc_name << "LBC_min";
		lbc_min.setName(lbc_name.str().c_str());
		mod2->add(lbc_min);
	}

	// Add LBC for z_ik variables -- version 1
	// if(k2 > sol.get_r()) k2 = sol.get_r();

	// IloArray<IloConstraint> const_array(env);
	// for(IloInt i = 0; i < n; i++){
	// 	IloExpr expr(env);
	// 	IloExpr expr2(env);
	// 	for(IloInt k = 0; k < n; k++)
	// 		if(sol.is_assigned(i, k))
	// 			expr += (1 - mod2->z[i][k]);
	// 		else
	// 			expr2 += mod2->z[i][k];

	// 	IloConstraint lbc2 = (expr + expr2 <= k2);
	// 	stringstream lbc2_name;
	// 	lbc2_name << "LBC2(" << i << ")";
	// 	lbc2.setName(lbc2_name.str().c_str());
	// 	mod2->add(lbc2);

	// 	const_array.add(lbc2);
	// }

	// Add LBC for z_ik variables -- version 2
	IloExpr aux(env);
	for(IloInt i = 0; i < n; i++){
		IloExpr expr(env);
		IloExpr expr2(env);
		for(IloInt k = 0; k < n; k++)
			if(sol.is_assigned(i, k))
				expr += (1 - mod2->z[i][k]);
			else
				expr2 += mod2->z[i][k];

		aux += expr + expr2;
	}

	IloConstraint lbc2 = (aux <= k2);
	stringstream lbc2_name;
	lbc2_name << "LBC2";
	lbc2.setName(lbc2_name.str().c_str());
	mod2->add(lbc2);

	// Run CPLEX on the current model
	IloCplex cplex(*mod2);
	set_cplex(cplex, ntl, UB, first);
	cplex.solve();

	// solver cplex(mod2);
	// cplex.run(ntl, UB, first);

	// Return the resulting solution
	bool to_ret = false;
	if(cplex.getStatus() == IloAlgorithm::Status::Optimal || cplex.getStatus() == IloAlgorithm::Status::Feasible){
		extract_model2(cplex);
		result = solution(sol.get_instance(), sol.get_p(), sol.get_r(), _z, _w, _x, _y, cplex.getObjValue());
		to_ret = true;
	}
	if(cplex.getStatus() == IloAlgorithm::Status::InfeasibleOrUnbounded || cplex.getStatus() == IloAlgorithm::Status::Unknown){
		result = sol;
		to_ret = false;
	}

	// Remove LBC's
	mod2->remove(lbc_max);
	mod2->remove(lbc2);
	// for(IloInt i = 0; i < const_array.getSize(); i++)
	// 	mod2->remove(const_array[i]);
	if(k_min != 0)
		mod2->remove(lbc_min);

	return to_ret;
}

/**
 * @brief Extracting variables from model 2 after solving
 * @details [long description]
 * 
 * @param cplex IloCplex solver for model 2
 */
void local_branching::extract_model2(IloCplex& cplex){
	int n = sol.get_instance().get_n();
	for(IloInt i = 0; i < n; i++){
		IloNumArray aux1(env);
		IloNumArray aux2(env);
		IloNumArray2 aux3(env);
		IloNumArray2 aux4(env);
		for(IloInt j = 0; j < n; j++){
			aux1.add(cplex.getValue(mod2->z[i][j]));
			aux2.add(cplex.getValue(mod2->w[i][j]));
			IloNumArray aux5(env);
			IloNumArray aux6(env);
			for(IloInt k = 0; k < n; k++){
				aux5.add(cplex.getValue(mod2->x[i][j][k]));
				aux6.add(cplex.getValue(mod2->y[i][j][k]));
			}
			aux3.add(aux5);
			aux4.add(aux6);
		}
		_z.add(aux1);
		_w.add(aux2);
		_x.add(aux3);
		_y.add(aux4);
	}
}

void local_branching::set_cplex( IloCplex& cplex, double ntl, double UB, bool first ) {
	cplex.setParam(IloCplex::Threads, 1);
	cplex.setParam(IloCplex::Param::Preprocessing::Aggregator, 0);
	cplex.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
	cplex.setParam(IloCplex::PreInd, IloFalse);
	cplex.setParam(IloCplex::Param::TimeLimit, ntl);
	cplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, UB);
	if(first)
		cplex.setParam(IloCplex::Param::MIP::Limits::Solutions, 1);
	cplex.setParam(IloCplex::Param::Emphasis::MIP, 1);
	cplex.exportModel("test.lp");
}