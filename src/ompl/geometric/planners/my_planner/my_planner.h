#ifndef OMPL_GEOMETRIC_PLANNERS_MYPLANNER_MYPLANNER_
#define OMPL_GEOMETRIC_PLANNERS_MYPLANNER_MYPLANNER_

#include <ompl/base/Planner.h>

// often useful headers:
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/geometric/planners/rrt/RRTstar_v2.h>

namespace ompl
{
    namespace geometric
    {
        class MyPlanner : public base::Planner
        {
        public:
            MyPlanner(const base::SpaceInformationPtr &si);

            virtual ~MyPlanner();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            // optional, if additional setup/configuration is needed, the setup() method can be implemented
            virtual void setup(void);

            virtual void getPlannerData(base::PlannerData &data) const;
        
            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

        private:
            RRTstarV2Ptr pPlanner_;
            double internal_planner_planning_time_;
        };
    } // namespace geometric
} // namespace ompl

#endif