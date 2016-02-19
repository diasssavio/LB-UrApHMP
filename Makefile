# System architecture
SYSTEM     = x86-64_linux

# Static library format for Cplex
LIBFORMAT  = static_pic

# Source code folder
SRC	= src
INCLUDE = include

# Machine hostname
MACHINE = $(shell hostname)

# Library type(STATIC or DYNAMIC)
MERGE = DYNAMIC

##### Folders
# Temp folders
TMP_ILS = ./tmp/LB_ILS
TMP_STATIC = ./tmp/lib/static
# Perm folders
DAT_DOXYFILE = ./dat/doxyfile
DAT_INSTANCES = ./dat/instances
DAT_LP_MODELS = ./dat/lp_models
DAT_RESULTS = ./dat/results


# Cplex directory
CPLEXDIR	  = /opt/ibm/ILOG/CPLEX_Studio1261/cplex

# Concert directory
CONCERTDIR	  = /opt/ibm/ILOG/CPLEX_Studio1261/concert

# Compiler
CCC = g++-4.8

# Compilation parameters (Add afterward: --coverage -pg -ftree-vectorize -mfpmath=sse -march=native)
CCOPT = -std=gnu++0x -O3 -ftree-vectorize -mfpmath=sse -march=native -march=native -flto -g -m64 -fPIC -fexceptions -DNDEBUG -DIL_STD

# Cplex static libraries directory
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

# Concert static libraries directory
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)

# Include libraries identifiers
CCLNFLAGS = -L$(CPLEXLIBDIR) -lilocplex -lcplex -L$(CONCERTLIBDIR) -lconcert -lm -pthread

# Cplex header's directory
CPLEXINCDIR   = $(CPLEXDIR)/include

# Concert header's directory
CONCERTINCDIR = $(CONCERTDIR)/include

# Header's include path
CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR)

# Executable name
CPP_EX = LB-ILS

# Compiling
all:
	mkdir -p $(TMP_ILS)
	mkdir -p $(TMP_STATIC)
	mkdir -p $(DAT_DOXYFILE)
	mkdir -p $(DAT_INSTANCES)
	mkdir -p $(DAT_LP_MODELS)
	mkdir -p $(DAT_RESULTS)
	make -j8 $(CPP_EX);

# Executing
execute: $(CPP_EX)
	./$(CPP_EX)

# Cleaning
clean:
	# /bin/rm -rf $(CPP_EX)
	/bin/rm -rf ./tmp
	/bin/rm -rf ./dat


########################## FRAMEWORK OBJECT's ######################################################
# CONFIG
# $(TMP_ILS)/FWConfig.o: $(SRC)/framework/config/FWConfig.cpp $(SRC)/framework/config/FWConfig.h
	# $(CCC) -c $(CCFLAGS) $(SRC)/framework/config/FWConfig.cpp -o $(TMP_ILS)/FWConfig.o

# $(TMP_ILS)/FWManager.o: $(SRC)/framework/config/FWManager.cpp $(SRC)/framework/config/FWManager.h
	# $(CCC) -c $(CCFLAGS) $(SRC)/framework/config/FWManager.cpp -o $(TMP_ILS)/FWManager.o

########################## GENERATING OBJECT's ######################################################

# CONFIGURATION - INSTANCES
$(TMP_ILS)/UrApHMP.o: $(SRC)/UrApHMP.cpp $(INCLUDE)/UrApHMP.h
	$(CCC) -c $(CCFLAGS) $(SRC)/UrApHMP.cpp -o $(TMP_ILS)/UrApHMP.o

# STRUCTURE - SOLUTION
$(TMP_ILS)/solution.o: $(SRC)/solution.cpp $(INCLUDE)/solution.h
	$(CCC) -c $(CCFLAGS) $(SRC)/solution.cpp -o $(TMP_ILS)/solution.o

# STRUCTURE - CHRONO
$(TMP_ILS)/FWChrono.o: $(SRC)/FWChrono.cpp $(INCLUDE)/FWChrono.h
	$(CCC) -c $(CCFLAGS) $(SRC)/FWChrono.cpp -o $(TMP_ILS)/FWChrono.o

# STRUCTURE - RANDOM GEN
$(TMP_ILS)/mt19937ar.o: $(SRC)/mt19937ar.c $(INCLUDE)/mt19937ar.h
	$(CCC) -c $(CCFLAGS) $(SRC)/mt19937ar.c -o $(TMP_ILS)/mt19937ar.o

# EXACT
$(TMP_ILS)/model.o: $(SRC)/model.cpp $(INCLUDE)/model.h
	$(CCC) -c $(CCFLAGS) $(SRC)/model.cpp -o $(TMP_ILS)/model.o
$(TMP_ILS)/solver.o: $(SRC)/solver.cpp $(INCLUDE)/solver.h
	$(CCC) -c $(CCFLAGS) $(SRC)/solver.cpp -o $(TMP_ILS)/solver.o

# BOOST
# $(TMP_ILS)/draw_graph.o: $(SRC)/draw_graph.cpp $(INCLUDE)/draw_graph.h
# 	$(CCC) -c $(CCFLAGS) $(SRC)/draw_graph.cpp -o $(TMP_ILS)/draw_graph.o

# LOCAL
$(TMP_ILS)/local_branching.o: $(SRC)/local_branching.cpp $(INCLUDE)/local_branching.h
	$(CCC) -c $(CCFLAGS) $(SRC)/local_branching.cpp -o $(TMP_ILS)/local_branching.o

# ILS
$(TMP_ILS)/ils.o: $(SRC)/ils.cpp $(INCLUDE)/ils.h
	$(CCC) -c $(CCFLAGS) $(SRC)/ils.cpp -o $(TMP_ILS)/ils.o

# MAIN
$(TMP_ILS)/main.o: $(SRC)/main.cpp
	$(CCC) -c $(CCFLAGS) $(SRC)/main.cpp -o $(TMP_ILS)/main.o

########################## OBJECT's LIBRARIES #######################################################
# CONFIGURATION
$(TMP_ILS)/Configuration.o:  $(TMP_ILS)/UrApHMP.o
	gcc -Wl,-r  $(TMP_ILS)/UrApHMP.o -o $(TMP_ILS)/Configuration.o -nostdlib

# STRUCTURE
$(TMP_ILS)/Structure.o: $(TMP_ILS)/solution.o $(TMP_ILS)/FWChrono.o $(TMP_ILS)/mt19937ar.o
	gcc -Wl,-r $(TMP_ILS)/solution.o $(TMP_ILS)/FWChrono.o $(TMP_ILS)/mt19937ar.o -o $(TMP_ILS)/Structure.o -nostdlib

# EXACT
$(TMP_ILS)/Exact.o: $(TMP_ILS)/model.o $(TMP_ILS)/solver.o
	gcc -Wl,-r $(TMP_ILS)/model.o $(TMP_ILS)/solver.o -o $(TMP_ILS)/Exact.o -nostdlib

# BOOST
# $(TMP_ILS)/Drawer.o: $(TMP_ILS)/draw_graph.o
# 	gcc -Wl,-r $(TMP_ILS)/draw_graph.o -o $(TMP_ILS)/Drawer.o -nostdlib

# LB
$(TMP_ILS)/LB.o: $(TMP_ILS)/local_branching.o
	gcc -Wl,-r $(TMP_ILS)/local_branching.o -o $(TMP_ILS)/LB.o -nostdlib

# ILS
$(TMP_ILS)/ILS.o: $(TMP_ILS)/ils.o
	gcc -Wl,-r $(TMP_ILS)/ils.o -o $(TMP_ILS)/ILS.o -nostdlib

########################## LINKANDO TUDO ########################################################
$(CPP_EX): $(TMP_ILS)/Exact.o $(TMP_ILS)/Configuration.o $(TMP_ILS)/Structure.o $(TMP_ILS)/LB.o $(TMP_ILS)/ILS.o $(TMP_ILS)/main.o
	$(CCC)  $(CCFLAGS) $(TMP_ILS)/Exact.o $(TMP_ILS)/Configuration.o $(TMP_ILS)/Structure.o $(TMP_ILS)/LB.o $(TMP_ILS)/ILS.o $(TMP_ILS)/main.o -L$(TMP_STATIC) -o $(CPP_EX) $(CCLNFLAGS)
#endif
