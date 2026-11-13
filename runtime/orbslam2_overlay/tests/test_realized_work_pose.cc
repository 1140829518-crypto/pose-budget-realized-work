#include "Optimizer.h"
#include "Frame.h"
#include "Map.h"
#include "MapPoint.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace ORB_SLAM2;

namespace {

struct Result
{
    int inliers;
    cv::Mat pose;
    std::vector<bool> outliers;
    std::vector<MapPointDynamicBelief> beliefs;
    double elapsedMs;
};

std::vector<std::string> Split(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while(std::getline(stream,field,','))
        fields.push_back(field);
    return fields;
}

std::vector<std::map<std::string,std::string> > ReadCsv(const std::string &path)
{
    std::ifstream input(path.c_str());
    assert(input.good());
    std::string line;
    assert(std::getline(input,line));
    const std::vector<std::string> header=Split(line);
    std::vector<std::map<std::string,std::string> > rows;
    while(std::getline(input,line))
    {
        if(line.empty())
            continue;
        const std::vector<std::string> fields=Split(line);
        assert(fields.size()==header.size());
        std::map<std::string,std::string> row;
        for(size_t i=0;i<header.size();++i)
            row[header[i]]=fields[i];
        rows.push_back(row);
    }
    return rows;
}

long long Integer(const std::map<std::string,std::string> &row,
                  const std::string &field)
{
    return std::strtoll(row.find(field)->second.c_str(),NULL,10);
}

double Real(const std::map<std::string,std::string> &row,
            const std::string &field)
{
    return std::strtod(row.find(field)->second.c_str(),NULL);
}

void RemoveLogSet(const std::string &path)
{
    std::remove(path.c_str());
    std::remove((path+".rounds.csv").c_str());
    std::remove((path+".io.csv").c_str());
    std::remove((path+".transport.csv").c_str());
    std::remove((path+".status.csv").c_str());
}

Result Run(const char *realizedWorkPath, unsigned long frameId,
           const char *callSource)
{
    if(realizedWorkPath)
        setenv("ORB_SLAM2_REALIZED_WORK_LOG",realizedWorkPath,1);
    else
        unsetenv("ORB_SLAM2_REALIZED_WORK_LOG");

    Frame frame;
    frame.mnId=frameId;
    frame.N=12;
    frame.fx=500.0f; frame.fy=500.0f;
    frame.cx=320.0f; frame.cy=240.0f; frame.mbf=40.0f;
    frame.mnScaleLevels=1;
    frame.mvScaleFactors.assign(1,1.0f);
    frame.mvInvLevelSigma2.assign(1,1.0f);
    frame.mvKeysUn.resize(frame.N);
    frame.mvuRight.assign(frame.N,-1.0f);
    frame.mvpMapPoints.assign(frame.N,static_cast<MapPoint*>(NULL));
    frame.mvMeasurementReliability.resize(frame.N);
    frame.mvbOutlier.assign(frame.N,false);
    frame.mDescriptors=cv::Mat::zeros(frame.N,32,CV_8U);
    cv::Mat initial=cv::Mat::eye(4,4,CV_32F);
    initial.at<float>(0,3)=0.015f;
    frame.SetPose(initial);

    Map map;
    std::vector<MapPoint*> points;
    for(int i=0;i<frame.N;++i)
    {
        const float x=(i%4-1.5f)*0.35f;
        const float y=(i/4-1.0f)*0.30f;
        const float z=4.0f+0.15f*i;
        frame.mvKeysUn[i]=cv::KeyPoint(frame.cx+frame.fx*x/z,
                                       frame.cy+frame.fy*y/z,1.0f);
        frame.mvKeysUn[i].octave=0;
        frame.mvMeasurementReliability[i]=0.35f+0.07f*i;
        cv::Mat position=(cv::Mat_<float>(3,1)<<x,y,z);
        MapPoint *point=new MapPoint(position,&map,&frame,i);
        MapPointDynamicBelief belief=point->GetDynamicBelief();
        belief.dynamic_probability=0.1f+0.05f*i;
        belief.uncertainty=0.2f+0.03f*i;
        belief.conflict=0.04f*i;
        belief.conflict_persistence=0.02f*i;
        point->SetDynamicBelief(belief);
        frame.mvpMapPoints[i]=point;
        points.push_back(point);
    }

    const std::chrono::steady_clock::time_point start=
        std::chrono::steady_clock::now();
    Result result;
    result.inliers=Optimizer::PoseOptimization(&frame,callSource);
    result.elapsedMs=std::chrono::duration_cast<
        std::chrono::duration<double,std::milli> >(
            std::chrono::steady_clock::now()-start).count();
    result.pose=frame.mTcw.clone();
    result.outliers=frame.mvbOutlier;
    for(size_t i=0;i<points.size();++i)
    {
        result.beliefs.push_back(points[i]->GetDynamicBelief());
        delete points[i];
    }
    return result;
}

void CheckEquivalent(const Result &left, const Result &right)
{
    assert(left.inliers==right.inliers);
    assert(left.outliers==right.outliers);
    assert(cv::norm(left.pose-right.pose)<1e-9);
    assert(left.beliefs.size()==right.beliefs.size());
    for(size_t i=0;i<left.beliefs.size();++i)
    {
        assert(left.beliefs[i].dynamic_probability==
               right.beliefs[i].dynamic_probability);
        assert(left.beliefs[i].uncertainty==right.beliefs[i].uncertainty);
        assert(left.beliefs[i].conflict==right.beliefs[i].conflict);
        assert(left.beliefs[i].conflict_persistence==
               right.beliefs[i].conflict_persistence);
    }
}

void RunEarlyReturn(const std::string &path)
{
    setenv("ORB_SLAM2_REALIZED_WORK_LOG",path.c_str(),1);
    Frame frame;
    frame.mnId=99;
    frame.N=0;
    frame.mvpMapPoints.clear();
    frame.mvbOutlier.clear();
    frame.SetPose(cv::Mat::eye(4,4,CV_32F));
    assert(Optimizer::PoseOptimization(&frame,"SYNTHETIC_EARLY_RETURN")==0);
}

void CheckInvocationAndRoundCsv(const std::string &path)
{
    const std::vector<std::map<std::string,std::string> > invocations=
        ReadCsv(path);
    const std::vector<std::map<std::string,std::string> > rounds=
        ReadCsv(path+".rounds.csv");
    assert(invocations.size()==3);
    assert(rounds.size()==12);
    assert(invocations[0].find("schema_version")->second=="REALIZED_LM_WORK_V2");
    assert(Integer(invocations[0],"optimizer_invocation_id")!=
           Integer(invocations[1],"optimizer_invocation_id"));
    assert(Integer(invocations[0],"frame_id")==42);
    assert(Integer(invocations[1],"frame_id")==42);
    assert(invocations[0].find("call_source")->second=="SYNTHETIC_TRACKING");
    assert(invocations[1].find("call_source")->second=="SYNTHETIC_RELOCALIZATION");
    assert(invocations[2].find("status")->second==
           "INSUFFICIENT_CORRESPONDENCES");
    assert(Integer(invocations[2],"executed_rounds")==0);

    for(size_t invocationIndex=0;invocationIndex<2;++invocationIndex)
    {
        const long long invocationId=
            Integer(invocations[invocationIndex],"optimizer_invocation_id");
        long long requested=0;
        long long outerStarted=0;
        long long outerCompleted=0;
        long long trials=0;
        long long accepted=0;
        long long rejected=0;
        int count=0;
        for(size_t i=0;i<rounds.size();++i)
        {
            if(Integer(rounds[i],"optimizer_invocation_id")!=invocationId)
                continue;
            ++count;
            assert(Integer(rounds[i],"executed")==1);
            assert(Integer(rounds[i],"round")==count-1);
            assert(Integer(rounds[i],"active_vertices")==1);
            assert(Integer(rounds[i],"hessian_dimension")==6);
            requested+=Integer(rounds[i],"requested");
            outerStarted+=Integer(rounds[i],"outer_started");
            outerCompleted+=Integer(rounds[i],"outer_completed");
            trials+=Integer(rounds[i],"lm_trials");
            accepted+=Integer(rounds[i],"accepted_trials");
            rejected+=Integer(rounds[i],"rejected_trials");
            assert(Integer(rounds[i],"lm_trials")==
                   Integer(rounds[i],"accepted_trials")+
                   Integer(rounds[i],"rejected_trials"));
            assert(Integer(rounds[i],"linear_solve_attempts")==
                   Integer(rounds[i],"lm_trials"));
            assert(std::isfinite(Real(rounds[i],"initial_robust_chi2")));
            assert(std::isfinite(Real(rounds[i],"final_robust_chi2")));
            assert(std::isfinite(Real(rounds[i],"terminal_damping")));
            assert(std::isfinite(Real(rounds[i],"objective_reduction")));
            assert(std::fabs(Real(rounds[i],"objective_reduction")-
                   (Real(rounds[i],"initial_robust_chi2")-
                    Real(rounds[i],"final_robust_chi2")))<1e-9);
        }
        assert(count==4);
        assert(requested==Integer(invocations[invocationIndex],
                                  "total_requested_outer_iterations"));
        assert(outerStarted==Integer(invocations[invocationIndex],
                                     "total_outer_started"));
        assert(outerCompleted==Integer(invocations[invocationIndex],
                                       "total_outer_completed"));
        assert(trials==Integer(invocations[invocationIndex],"total_lm_trials"));
        assert(accepted==Integer(invocations[invocationIndex],
                                 "total_accepted_trials"));
        assert(rejected==Integer(invocations[invocationIndex],
                                 "total_rejected_trials"));
    }

    const long long earlyId=Integer(invocations[2],"optimizer_invocation_id");
    int earlyRoundCount=0;
    for(size_t i=0;i<rounds.size();++i)
    {
        if(Integer(rounds[i],"optimizer_invocation_id")!=earlyId)
            continue;
        ++earlyRoundCount;
        assert(Integer(rounds[i],"executed")==0);
    }
    assert(earlyRoundCount==4);
}

void CheckScientificCsvEquivalent(const std::string &syncPath,
                                  const std::string &bufferedPath,
                                  const bool expectTransport=true)
{
    const std::vector<std::map<std::string,std::string> > syncInv=ReadCsv(syncPath);
    const std::vector<std::map<std::string,std::string> > bufferedInv=ReadCsv(bufferedPath);
    const std::vector<std::map<std::string,std::string> > syncRounds=
        ReadCsv(syncPath+".rounds.csv");
    const std::vector<std::map<std::string,std::string> > bufferedRounds=
        ReadCsv(bufferedPath+".rounds.csv");
    assert(syncInv.size()==bufferedInv.size());
    assert(syncRounds.size()==bufferedRounds.size());
    const char *invIgnored[]={"optimizer_invocation_id","total_solver_core_ms",
                             "pose_optimization_total_ms"};
    const char *roundIgnored[]={"optimizer_invocation_id","optimize_ms"};
    for(size_t i=0;i<syncInv.size();++i)
    {
        for(std::map<std::string,std::string>::const_iterator it=syncInv[i].begin();
            it!=syncInv[i].end();++it)
        {
            bool ignored=false;
            for(size_t j=0;j<sizeof(invIgnored)/sizeof(invIgnored[0]);++j)
                ignored=ignored || it->first==invIgnored[j];
            if(!ignored)
                assert(bufferedInv[i].find(it->first)->second==it->second);
        }
    }
    for(size_t i=0;i<syncRounds.size();++i)
    {
        for(std::map<std::string,std::string>::const_iterator it=syncRounds[i].begin();
            it!=syncRounds[i].end();++it)
        {
            bool ignored=false;
            for(size_t j=0;j<sizeof(roundIgnored)/sizeof(roundIgnored[0]);++j)
                ignored=ignored || it->first==roundIgnored[j];
            if(!ignored)
                assert(bufferedRounds[i].find(it->first)->second==it->second);
        }
    }
    if(!expectTransport)
        return;
    const std::vector<std::map<std::string,std::string> > transport=
        ReadCsv(bufferedPath+".transport.csv");
    assert(transport.size()==bufferedInv.size());
    for(size_t i=0;i<transport.size();++i)
    {
        assert(transport[i].find("logger_mode")->second=="BUFFERED");
        assert(Integer(transport[i],"write_ok")==1);
        assert(Real(transport[i],"producer_enqueue_ms")>=0.0);
        assert(Real(transport[i],"consumer_write_ms")>=0.0);
        assert(Integer(transport[i],"optimizer_invocation_id")==
               Integer(bufferedInv[i],"optimizer_invocation_id"));
    }
}

} // namespace

int main()
{
    setenv("ORB_SLAM2_MEASUREMENT_POLICY","BELIEF_ONLY",1);
    setenv("ORB_SLAM2_POSE_BUDGET_MODE","A",1);
    unsetenv("ORB_SLAM2_OPTIMIZER_MEASUREMENT_LOG");
    unsetenv("ORB_SLAM2_POSE_BUDGET_LOG");
    unsetenv("ORB_SLAM2_POSE_ITERATION_LOG");

    const std::string path="/tmp/orb_slam2_realized_work_pose.csv";
    RemoveLogSet(path);
    setenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE","SYNC",1);
    const Result off=Run(NULL,42,"SYNTHETIC_TRACKING");
    const Result on=Run(path.c_str(),42,"SYNTHETIC_TRACKING");
    const Result sameFrame=Run(path.c_str(),42,"SYNTHETIC_RELOCALIZATION");
    CheckEquivalent(off,on);
    CheckEquivalent(on,sameFrame);
    RunEarlyReturn(path);
    CheckInvocationAndRoundCsv(path);

    const std::string countOnlyPath="/tmp/orb_slam2_realized_work_pose_count.csv";
    RemoveLogSet(countOnlyPath);
    setenv("ORB_SLAM2_REALIZED_WORK_MODE","COUNT_ONLY",1);
    const Result countFirst=Run(countOnlyPath.c_str(),42,"SYNTHETIC_TRACKING");
    const Result countSecond=Run(countOnlyPath.c_str(),42,"SYNTHETIC_RELOCALIZATION");
    RunEarlyReturn(countOnlyPath);
    assert(!std::ifstream(countOnlyPath.c_str()).good());
    Optimizer::FlushRealizedWorkLog();
    CheckEquivalent(on,countFirst);
    CheckEquivalent(sameFrame,countSecond);
    CheckInvocationAndRoundCsv(countOnlyPath);
    CheckScientificCsvEquivalent(path,countOnlyPath,false);
    const std::vector<std::map<std::string,std::string> > countStatus=
        ReadCsv(countOnlyPath+".status.csv");
    assert(countStatus.size()==1);
    assert(countStatus[0].find("mode")->second=="COUNT_ONLY");
    assert(Integer(countStatus[0],"complete")==1);
    assert(Integer(countStatus[0],"fixed_store_overflow")==0);
    assert(Integer(countStatus[0],"path_mismatch")==0);
    assert(Integer(countStatus[0],"transport_failure")==0);
    assert(!std::ifstream((countOnlyPath+".transport.csv").c_str()).good());

    const std::string bufferedPath="/tmp/orb_slam2_realized_work_pose_buffered.csv";
    RemoveLogSet(bufferedPath);
    setenv("ORB_SLAM2_REALIZED_WORK_MODE","BUFFERED",1);
    setenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE","BUFFERED",1);
    const Result bufferedFirst=Run(bufferedPath.c_str(),42,"SYNTHETIC_TRACKING");
    const Result bufferedSecond=Run(bufferedPath.c_str(),42,"SYNTHETIC_RELOCALIZATION");
    RunEarlyReturn(bufferedPath);
    Optimizer::FlushRealizedWorkLog();
    CheckEquivalent(on,bufferedFirst);
    CheckEquivalent(sameFrame,bufferedSecond);
    CheckInvocationAndRoundCsv(bufferedPath);
    CheckScientificCsvEquivalent(path,bufferedPath);
    const std::vector<std::map<std::string,std::string> > bufferedStatus=
        ReadCsv(bufferedPath+".status.csv");
    assert(bufferedStatus.size()==1);
    assert(bufferedStatus[0].find("mode")->second=="BUFFERED");
    assert(Integer(bufferedStatus[0],"complete")==1);
    assert(Integer(bufferedStatus[0],"transport_failure")==0);

    const std::string overheadPath="/tmp/orb_slam2_realized_work_overhead.csv";
    RemoveLogSet(overheadPath);
    setenv("ORB_SLAM2_REALIZED_WORK_MODE","SYNC",1);
    setenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE","SYNC",1);
    const int repetitions=200;
    double offMs=0.0;
    double onMs=0.0;
    for(int i=0;i<repetitions;++i)
        offMs+=Run(NULL,1000+i,"OVERHEAD_OFF").elapsedMs;
    for(int i=0;i<repetitions;++i)
        onMs+=Run(overheadPath.c_str(),2000+i,"OVERHEAD_ON").elapsedMs;
    offMs/=repetitions;
    onMs/=repetitions;
    const std::vector<std::map<std::string,std::string> > overheadRows=
        ReadCsv(overheadPath);
    const std::vector<std::map<std::string,std::string> > ioRows=
        ReadCsv(overheadPath+".io.csv");
    assert(static_cast<int>(overheadRows.size())==repetitions);
    assert(static_cast<int>(ioRows.size())==repetitions);
    double preLogMs=0.0;
    double payloadWriteMs=0.0;
    for(int i=0;i<repetitions;++i)
    {
        preLogMs+=Real(overheadRows[i],"pose_optimization_total_ms");
        payloadWriteMs+=Real(ioRows[i],"payload_write_ms");
        assert(Integer(ioRows[i],"write_ok")==1);
    }
    preLogMs/=repetitions;
    payloadWriteMs/=repetitions;

    const std::string bufferedOverheadPath=
        "/tmp/orb_slam2_realized_work_buffered_overhead.csv";
    RemoveLogSet(bufferedOverheadPath);
    setenv("ORB_SLAM2_REALIZED_WORK_MODE","BUFFERED",1);
    setenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE","BUFFERED",1);
    double bufferedMs=0.0;
    for(int i=0;i<repetitions;++i)
        bufferedMs+=Run(bufferedOverheadPath.c_str(),3000+i,
                        "BUFFERED_OVERHEAD").elapsedMs;
    Optimizer::FlushRealizedWorkLog();
    bufferedMs/=repetitions;
    const std::vector<std::map<std::string,std::string> > transportRows=
        ReadCsv(bufferedOverheadPath+".transport.csv");
    assert(static_cast<int>(transportRows.size())==repetitions);
    double producerMs=0.0;
    double consumerMs=0.0;
    for(int i=0;i<repetitions;++i)
    {
        producerMs+=Real(transportRows[i],"producer_enqueue_ms");
        consumerMs+=Real(transportRows[i],"consumer_write_ms");
        assert(Integer(transportRows[i],"write_ok")==1);
    }
    producerMs/=repetitions;
    consumerMs/=repetitions;

    const std::string overflowPath=
        "/tmp/orb_slam2_realized_work_count_overflow.csv";
    RemoveLogSet(overflowPath);
    setenv("ORB_SLAM2_REALIZED_WORK_MODE","COUNT_ONLY",1);
    setenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE","COUNT_ONLY",1);
    for(int i=0;i<1025;++i)
        RunEarlyReturn(overflowPath);
    Optimizer::FlushRealizedWorkLog();
    const std::vector<std::map<std::string,std::string> > overflowInvocations=
        ReadCsv(overflowPath);
    const std::vector<std::map<std::string,std::string> > overflowRounds=
        ReadCsv(overflowPath+".rounds.csv");
    const std::vector<std::map<std::string,std::string> > overflowStatus=
        ReadCsv(overflowPath+".status.csv");
    assert(overflowInvocations.size()==1024);
    assert(overflowRounds.size()==4096);
    assert(overflowStatus.size()==1);
    assert(Integer(overflowStatus[0],"complete")==0);
    assert(Integer(overflowStatus[0],"fixed_store_overflow")==1);
    assert(Integer(overflowStatus[0],"transport_failure")==0);
    assert(!std::ifstream((overflowPath+".transport.csv").c_str()).good());
    std::cout << "realized-work PoseOptimization noninterference: PASS\n"
              << "overhead repetitions=" << repetitions
              << " off_mean_ms=" << offMs
              << " instrumented_prelog_mean_ms=" << preLogMs
              << " payload_write_mean_ms=" << payloadWriteMs
              << " on_mean_ms=" << onMs
              << " delta_ms=" << onMs-offMs << '\n'
              << "buffered_mean_ms=" << bufferedMs
              << " buffered_delta_ms=" << bufferedMs-offMs
              << " producer_enqueue_mean_ms=" << producerMs
              << " consumer_write_mean_ms=" << consumerMs << std::endl;

    RemoveLogSet(path);
    RemoveLogSet(countOnlyPath);
    RemoveLogSet(bufferedPath);
    RemoveLogSet(overheadPath);
    RemoveLogSet(bufferedOverheadPath);
    RemoveLogSet(overflowPath);
    unsetenv("ORB_SLAM2_REALIZED_WORK_LOG");
    unsetenv("ORB_SLAM2_REALIZED_WORK_MODE");
    unsetenv("ORB_SLAM2_REALIZED_WORK_LOG_MODE");
    return 0;
}
