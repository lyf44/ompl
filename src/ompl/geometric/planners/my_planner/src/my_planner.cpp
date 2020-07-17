#include <limits>

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/geometric/planners/my_planner/my_planner.h"

ompl::geometric::MyPlanner::MyPlanner(const base::SpaceInformationPtr &si) : 
    base::Planner(si, "my_planner")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    // create an internal planner
    base::SpaceInformationPtr si_copy = si_; // copy the space information. This is because in base planner constructor, si is MOVED
                                             // into si_!!!
    pPlanner_ = std::make_shared<RRTstarV2>(si_copy);
    
    // settings
    internal_planner_planning_time_ = 1.0;
    // goalBias_ = 0.05;
    // maxDistance_ = 0.0;
    // lastGoalMotion_ = NULL;

    // Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
    // Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
}

ompl::geometric::MyPlanner::~MyPlanner()
{
    // delete pPlanner_;
    // freeMemory();
}

void ompl::geometric::MyPlanner::clear(void)
{
    Planner::clear();
    pPlanner_->clear();
    // sampler_.reset();
    // freeMemory();
    // if (nn_)
    //     nn_->clear();
    // lastGoalMotion_ = NULL;
}

void ompl::geometric::MyPlanner::setup(void)
{
    Planner::setup();

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // tools::SelfConfig sc(si_, getName());
    // sc.configurePlannerRange(maxDistance_);

    // if (!nn_)
    //     nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(si_->getStateSpace()));
    // nn_->setDistanceFunction(boost::bind(&RRT::distanceFunction, this, _1, _2));
}

// void ompl::geometric::MyPlanner::freeMemory(void)
// {

// }

ompl::base::PlannerStatus ompl::geometric::MyPlanner::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("MyPlanner::solve");
    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkValidity();

    // call internal planner to try to solve the problem within designated time.
    ompl::base::PlannerStatus status = pPlanner_->solve(base::timedPlannerTerminationCondition(internal_planner_planning_time_));
    OMPL_INFORM("MyPlanner::solve finished");

    ompl::geometric::RRTstarV2::Motion* pBestMotion = NULL;
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
        // if exact solution is found, best motion is best goal motion
        pBestMotion = pPlanner_->getBestGoalMotion();
        OMPL_INFORM("MyPlanner: best motion retrieved from internal planner");
    } else {
        // get planner data
        base::PlannerData plannerData(si_);
        pPlanner_->getPlannerData(plannerData);
        unsigned int numVertices = plannerData.numVertices();
        unsigned int numEdges = plannerData.numEdges();
        unsigned int numStartVertices = plannerData.numStartVertices();
        OMPL_INFORM("MyPlanner: planner num of vertices = %d", numVertices);
        OMPL_INFORM("MyPlanner: planner num of edges = %d", numEdges);
        OMPL_INFORM("MyPlanner: planner num of start vertices = %d", numStartVertices);

        // get motions
        std::vector<ompl::geometric::RRTstarV2::Motion*> pMotions;
        OMPL_INFORM("MyPlanner: planner get motions");
        pPlanner_->getMotions(pMotions);
        OMPL_INFORM("MyPlanner: motions size = %d", pMotions.size());

        // for all vertices sampled, find the best vertices
        ompl::base::GoalPtr pGoal = pdef_->getGoal();

        double best_cost = std::numeric_limits<double>::max();
        for (auto pMotion: pMotions) {
            if (pMotion->parent == nullptr) continue; // filter out start motion
            // double cost_from_source = pMotion->cost.value();
            // double cost_to_go = 0.0;
            // pGoal->isSatisfied(pMotion->state, &cost_to_go);
            // double total_cost = cost_from_source + cost_to_go;
            const base::Cost cost_to_come = pMotion->cost;
            const base::Cost cost_to_go = opt_->costToGo(pMotion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
            double total_cost = opt_->combineCosts(cost_to_come, cost_to_go).value();
            if (total_cost < best_cost) {
                best_cost = total_cost;
                pBestMotion = pMotion;
            }
        }
        OMPL_INFORM("MyPlanner: best motion found with cost %f", best_cost);
    }

    // construct the solution path
    std::vector<ompl::geometric::RRTstarV2::Motion*> mpath;
    ompl::geometric::RRTstarV2::Motion* pMotion = pBestMotion;
    while (pMotion != nullptr)
    {
        mpath.push_back(pMotion);
        pMotion = pMotion->parent;
    }

    // set the solution path
    auto path(std::make_shared<PathGeometric>(si_));
    for (int i = mpath.size() - 1; i >= 0; --i)
        path->append(mpath[i]->state);

    // Add the solution path.
    base::PlannerSolution psol(path);
    psol.setPlannerName(getName());

    // If we don't have a goal motion, the solution is approximate
    // if (!bestGoalMotion_)
    //     psol.setApproximate(approxDist);

    // Does the solution satisfy the optimization objective?
    // psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
    pdef_->addSolutionPath(psol);

    return base::PlannerStatus(true, status != ompl::base::PlannerStatus::EXACT_SOLUTION);
}

void ompl::geometric::MyPlanner::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // std::vector<Motion *> motions;
    // if (nn_)
    //     nn_->list(motions);

    // if (lastGoalMotion_)
    //     data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    // for (unsigned int i = 0; i < motions.size(); ++i)
    // {
    //     if (motions[i]->parent == NULL)
    //         data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
    //     else
    //         data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
    //                      base::PlannerDataVertex(motions[i]->state));
    // }
}

void ompl::geometric::MyPlanner::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
    pPlanner_->setProblemDefinition(pdef);
    pis_.update();
}


/*
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc)
        {
            // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
            // ensures that there is at least one input state and a ompl::base::Goal object specified
            checkValidity();
 
            // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
            base::Goal *goal = pdef_->getGoal().get();
 
            // get input states with PlannerInputStates helper, pis_
            while (const base::State *st = pis_.nextStart())
            {
                // st will contain a start state.  Typically this state will
                // be cloned here and inserted into the Planner's data structure.
            }
 
            // if needed, sample states from the goal region (and wait until a state is sampled)
            const base::State *st = pis_.nextGoal(ptc);
            // or sample a new goal state only if available:
            const base::State *st = pis_.nextGoal();
 
            // periodically check if ptc() returns true.
            // if it does, terminate planning.
            while (ptc() == false)
            {
                // Start planning here.
 
                // call routines from SpaceInformation (si_) as needed. i.e.,
                // si_->allocStateSampler() for sampling,
                // si_->checkMotion(state1, state2) for state validity, etc...
 
                // use the Goal pointer to evaluate whether a sampled state satisfies the goal requirements
 
                // use log macros for informative messaging, i.e., logInfo("Planner found a solution!");
            }
 
            // When a solution path is computed, save it here
            pdef_->addSolutionPath(...);
 
            // Return a value from the PlannerStatus enumeration.
            // See ompl::base::PlannerStatus for the possible return values
            return base::PlannerStatus::EXACT_SOLUTION;
        }
*/