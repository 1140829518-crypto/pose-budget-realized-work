#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"

#include <Eigen/Core>

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

class ScalarVertex : public g2o::BaseVertex<1, double>
{
public:
    ScalarVertex() { setEstimate(0.0); }
    virtual bool read(std::istream &) { return false; }
    virtual bool write(std::ostream &) const { return false; }
protected:
    virtual void setToOriginImpl() { _estimate=0.0; }
    virtual void oplusImpl(const double *update) { _estimate+=update[0]; }
};

class ScalarEdge : public g2o::BaseUnaryEdge<1, double, ScalarVertex>
{
public:
    ScalarEdge()
    {
        setMeasurement(10.0);
        information()(0,0)=1.0;
    }
    virtual void computeError()
    {
        const ScalarVertex *vertex=static_cast<const ScalarVertex*>(_vertices[0]);
        _error[0]=vertex->estimate()-_measurement;
    }
    virtual void linearizeOplus() { _jacobianOplusXi(0,0)=1.0; }
    virtual bool read(std::istream &) { return false; }
    virtual bool write(std::ostream &) const { return false; }
};

class ScriptedAlgorithm : public g2o::OptimizationAlgorithm
{
public:
    ScriptedAlgorithm(bool initOk, const std::vector<SolverResult> &results)
        : initOk_(initOk), results_(results), calls_(0) {}
    virtual bool init(bool) { return initOk_; }
    virtual SolverResult solve(int, bool)
    {
        const size_t index=static_cast<size_t>(calls_++);
        return index<results_.size()?results_[index]:OK;
    }
    virtual bool computeMarginals(
        g2o::SparseBlockMatrix<Eigen::MatrixXd> &,
        const std::vector<std::pair<int,int> > &) { return false; }
    virtual bool updateStructure(
        const std::vector<g2o::HyperGraph::Vertex*> &,
        const g2o::HyperGraph::EdgeSet &) { return true; }
private:
    bool initOk_;
    std::vector<SolverResult> results_;
    int calls_;
};

class ScriptedSolver : public g2o::Solver
{
public:
    ScriptedSolver(bool initOk, bool buildStructureOk, bool solveOk,
                   double step)
        : initOk_(initOk), buildStructureOk_(buildStructureOk),
          solveOk_(solveOk), step_(step), schur_(false), writeDebug_(false) {}
    virtual bool init(g2o::SparseOptimizer *optimizer, bool)
    {
        setOptimizer(optimizer);
        if(!initOk_)
            return false;
        resizeVector(1);
        _x[0]=step_;
        _b[0]=10.0;
        return true;
    }
    virtual bool buildStructure(bool=false) { return buildStructureOk_; }
    virtual bool updateStructure(const std::vector<g2o::HyperGraph::Vertex*> &,
                                 const g2o::HyperGraph::EdgeSet &) { return true; }
    virtual bool buildSystem()
    {
        _x[0]=step_;
        _b[0]=10.0;
        return true;
    }
    virtual bool solve()
    {
        _x[0]=step_;
        return solveOk_;
    }
    virtual bool computeMarginals(
        g2o::SparseBlockMatrix<Eigen::MatrixXd> &,
        const std::vector<std::pair<int,int> > &) { return false; }
    virtual bool setLambda(double, bool=false) { return true; }
    virtual void restoreDiagonal() {}
    virtual bool schur() { return schur_; }
    virtual void setSchur(bool value) { schur_=value; }
    virtual void setWriteDebug(bool value) { writeDebug_=value; }
    virtual bool writeDebug() const { return writeDebug_; }
    virtual bool saveHessian(const std::string &) const { return false; }
private:
    bool initOk_;
    bool buildStructureOk_;
    bool solveOk_;
    double step_;
    bool schur_;
    bool writeDebug_;
};

void AddScalarGraph(g2o::SparseOptimizer &optimizer)
{
    ScalarVertex *vertex=new ScalarVertex;
    vertex->setId(0);
    assert(optimizer.addVertex(vertex));
    ScalarEdge *edge=new ScalarEdge;
    edge->setId(0);
    edge->setVertex(0,vertex);
    assert(optimizer.addEdge(edge));
    assert(optimizer.initializeOptimization());
}

void TestSparseRequestedMaximum()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    optimizer.setAlgorithm(new ScriptedAlgorithm(true,
        std::vector<g2o::OptimizationAlgorithm::SolverResult>(3,
            g2o::OptimizationAlgorithm::OK)));
    AddScalarGraph(optimizer);
    assert(optimizer.optimize(3)==3);
    const g2o::SparseOptimizer::OptimizationRunStatistics &stats=
        optimizer.lastOptimizationRunStatistics();
    assert(stats.requestedIterations==3);
    assert(stats.outerIterationsStarted==3);
    assert(stats.outerIterationsCompleted==3);
    assert(stats.rawReturnValue==3);
    assert(stats.capReached);
    assert(stats.stopReason==g2o::SparseOptimizer::OptimizationRequestedMaximum);
}

void TestSparseNaturalTermination()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    std::vector<g2o::OptimizationAlgorithm::SolverResult> results;
    results.push_back(g2o::OptimizationAlgorithm::OK);
    results.push_back(g2o::OptimizationAlgorithm::Terminate);
    optimizer.setAlgorithm(new ScriptedAlgorithm(true,results));
    AddScalarGraph(optimizer);
    assert(optimizer.optimize(8)==2);
    const g2o::SparseOptimizer::OptimizationRunStatistics &stats=
        optimizer.lastOptimizationRunStatistics();
    assert(stats.outerIterationsStarted==2);
    assert(stats.outerIterationsCompleted==2);
    assert(stats.rawReturnValue==2);
    assert(!stats.capReached);
    assert(stats.stopReason==g2o::SparseOptimizer::OptimizationAlgorithmTerminate);
}

void TestSparseSolveFailure()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    std::vector<g2o::OptimizationAlgorithm::SolverResult> results(
        1,g2o::OptimizationAlgorithm::Fail);
    optimizer.setAlgorithm(new ScriptedAlgorithm(true,results));
    AddScalarGraph(optimizer);
    assert(optimizer.optimize(5)==0);
    const g2o::SparseOptimizer::OptimizationRunStatistics &stats=
        optimizer.lastOptimizationRunStatistics();
    assert(stats.outerIterationsStarted==1);
    assert(stats.outerIterationsCompleted==1);
    assert(stats.rawReturnValue==0);
    assert(stats.stopReason==g2o::SparseOptimizer::OptimizationAlgorithmFail);
}

void TestSparseInitializationFailure()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    optimizer.setAlgorithm(new ScriptedAlgorithm(false,
        std::vector<g2o::OptimizationAlgorithm::SolverResult>()));
    AddScalarGraph(optimizer);
    assert(optimizer.optimize(5)==-1);
    const g2o::SparseOptimizer::OptimizationRunStatistics &stats=
        optimizer.lastOptimizationRunStatistics();
    assert(stats.outerIterationsStarted==0);
    assert(stats.outerIterationsCompleted==0);
    assert(stats.rawReturnValue==-1);
    assert(stats.stopReason==g2o::SparseOptimizer::OptimizationInitializationFailure);
}

void TestSparseEmptyGraph()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    optimizer.setAlgorithm(new ScriptedAlgorithm(true,
        std::vector<g2o::OptimizationAlgorithm::SolverResult>()));
    assert(optimizer.optimize(5)==-1);
    const g2o::SparseOptimizer::OptimizationRunStatistics &stats=
        optimizer.lastOptimizationRunStatistics();
    assert(stats.rawReturnValue==-1);
    assert(stats.stopReason==g2o::SparseOptimizer::OptimizationEmptyActiveGraph);
}

struct LmFixture
{
    g2o::SparseOptimizer optimizer;
    g2o::OptimizationAlgorithmLevenberg *algorithm;
    LmFixture(bool initOk, bool buildStructureOk, bool solveOk, double step)
    {
        ScriptedSolver *solver=new ScriptedSolver(
            initOk,buildStructureOk,solveOk,step);
        algorithm=new g2o::OptimizationAlgorithmLevenberg(solver);
        algorithm->setUserLambdaInit(1.0);
        algorithm->setWorkInstrumentationEnabled(true);
        optimizer.setAlgorithm(algorithm);
        optimizer.setWorkInstrumentationEnabled(true);
        AddScalarGraph(optimizer);
    }
};

void TestLmAcceptedCap()
{
    LmFixture fixture(true,true,true,1.0);
    assert(fixture.optimizer.optimize(2)==2);
    const g2o::OptimizationAlgorithmLevenberg::WorkStatistics &stats=
        fixture.algorithm->workStatistics();
    assert(stats.lmTrials==2);
    assert(stats.acceptedTrials==2);
    assert(stats.rejectedTrials==0);
    assert(stats.linearSolveAttempts==2);
    assert(stats.linearSolveSuccesses==2);
    assert(stats.linearSolveFailures==0);
    assert(stats.residualSweeps==4);
    assert(stats.residualEdgeEvaluations==4);
    assert(stats.jacobianBuilds==2);
    assert(stats.jacobianEdgeEvaluations==2);
    assert(stats.hessianDimension==1);
    assert(std::isfinite(stats.initialRobustChi2));
    assert(std::isfinite(stats.finalRobustChi2));
    assert(std::isfinite(stats.terminalDamping));
    assert(std::isfinite(stats.objectiveReduction));
    assert(stats.terminalDamping==fixture.algorithm->currentLambda());
    assert(std::fabs(stats.objectiveReduction-
                     (stats.initialRobustChi2-stats.finalRobustChi2))<1e-12);
    assert(stats.objectiveReduction>0.0);
}

void TestLmZeroGain()
{
    LmFixture fixture(true,true,true,0.0);
    assert(fixture.optimizer.optimize(5)==1);
    const g2o::OptimizationAlgorithmLevenberg::WorkStatistics &stats=
        fixture.algorithm->workStatistics();
    assert(stats.lmTrials==1);
    assert(stats.acceptedTrials==0);
    assert(stats.rejectedTrials==1);
    assert(stats.linearSolveSuccesses==1);
    assert(stats.terminalDamping==fixture.algorithm->currentLambda());
    assert(stats.objectiveReduction==0.0);
    assert(fixture.algorithm->lastTerminationReason()==
        g2o::OptimizationAlgorithmLevenberg::TerminationZeroGain);
}

void TestLmMaximumTrialsAndLinearFailure()
{
    LmFixture fixture(true,true,false,0.0);
    fixture.algorithm->setMaxTrialsAfterFailure(3);
    assert(fixture.optimizer.optimize(5)==1);
    const g2o::OptimizationAlgorithmLevenberg::WorkStatistics &stats=
        fixture.algorithm->workStatistics();
    assert(stats.lmTrials==3);
    assert(stats.acceptedTrials==0);
    assert(stats.rejectedTrials==3);
    assert(stats.linearSolveAttempts==3);
    assert(stats.linearSolveSuccesses==0);
    assert(stats.linearSolveFailures==3);
    assert(stats.residualSweeps==4);
    assert(stats.terminalDamping==fixture.algorithm->currentLambda());
    assert(stats.objectiveReduction==0.0);
    assert(fixture.algorithm->lastTerminationReason()==
        g2o::OptimizationAlgorithmLevenberg::TerminationMaxTrials);
}

void TestLmSmallImprovement()
{
    LmFixture fixture(true,true,true,1e-6);
    assert(fixture.optimizer.optimize(10)==3);
    const g2o::OptimizationAlgorithmLevenberg::WorkStatistics &stats=
        fixture.algorithm->workStatistics();
    assert(stats.lmTrials==3);
    assert(stats.acceptedTrials==3);
    assert(stats.rejectedTrials==0);
    assert(stats.terminalDamping==fixture.algorithm->currentLambda());
    assert(std::fabs(stats.objectiveReduction-
                     (stats.initialRobustChi2-stats.finalRobustChi2))<1e-12);
    assert(fixture.algorithm->lastTerminationReason()==
        g2o::OptimizationAlgorithmLevenberg::TerminationSmallImprovement);
    assert(fixture.optimizer.lastOptimizationRunStatistics().stopReason==
        g2o::SparseOptimizer::OptimizationAlgorithmTerminate);
}

void TestLmBuildStructureFailure()
{
    LmFixture fixture(true,false,true,1.0);
    assert(fixture.optimizer.optimize(5)==0);
    const g2o::OptimizationAlgorithmLevenberg::WorkStatistics &stats=
        fixture.algorithm->workStatistics();
    assert(stats.lmTrials==0);
    assert(stats.residualSweeps==0);
    assert(!std::isfinite(stats.terminalDamping));
    assert(!std::isfinite(stats.objectiveReduction));
    assert(fixture.optimizer.lastOptimizationRunStatistics().stopReason==
        g2o::SparseOptimizer::OptimizationAlgorithmFail);
}

void TestLmInitializationFailure()
{
    LmFixture fixture(false,true,true,1.0);
    assert(fixture.optimizer.optimize(5)==-1);
    assert(fixture.optimizer.lastOptimizationRunStatistics().stopReason==
        g2o::SparseOptimizer::OptimizationInitializationFailure);
}

} // namespace

int main()
{
#if ORB_SLAM2_REALIZED_WORK_COUNTERS
    TestSparseRequestedMaximum();
    TestSparseNaturalTermination();
    TestSparseSolveFailure();
    TestSparseInitializationFailure();
    TestSparseEmptyGraph();
    TestLmAcceptedCap();
    TestLmZeroGain();
    TestLmMaximumTrialsAndLinearFailure();
    TestLmSmallImprovement();
    TestLmBuildStructureFailure();
    TestLmInitializationFailure();
    std::cout << "realized-work g2o synthetic counters: PASS" << std::endl;
#else
    g2o::SparseOptimizer optimizer;
    optimizer.setWorkInstrumentationEnabled(true);
    assert(!optimizer.workInstrumentationEnabled());
    ScriptedSolver *rawSolver=new ScriptedSolver(true,true,true,1.0);
    g2o::OptimizationAlgorithmLevenberg algorithm(rawSolver);
    algorithm.setWorkInstrumentationEnabled(true);
    assert(!algorithm.workInstrumentationEnabled());
    std::cout << "realized-work g2o compile-time OFF: PASS" << std::endl;
#endif
    return 0;
}
