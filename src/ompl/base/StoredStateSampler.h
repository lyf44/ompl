/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include "ompl/base/StateSampler.h"
#include <vector>

namespace ompl
{
    namespace base
    {

        /** \brief State space sampler for discrete states */
        class StoredStateSampler : public StateSampler
        {
        public:

            /** \brief Constructor. Takes the state space to be sampled (\e space) and the set of states to draw samples from (\e states) */
            StoredStateSampler(const StateSpace *space, const std::vector<const State*> &states);

            virtual void sampleUniform(State *state);
            virtual void sampleUniformNear(State *state, const State *near, const double distance);
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

            /** \brief When calling sampleUniformNear() or sampleGaussian(), multiple states are drawn uniformly
                at random, in an attempt to satisfy the requested distance. This function returns the number of attempts. */
            unsigned int getMaxNearSamplesAttempts(void) const
            {
                return maxNearSamplesAttempts_;
            }

            /** \brief When calling sampleUniformNear() or sampleGaussian(), multiple states are drawn uniformly
                at random, in an attempt to satisfy the requested distance. This function sets the number of attempts. */
            void setMaxNearSamplesAttempts(unsigned int maxNearSamplesAttempts)
            {
                if (maxNearSamplesAttempts > 0)
                    maxNearSamplesAttempts_ = maxNearSamplesAttempts;
            }

        protected:

            /** \brief The states to sample from */
            const std::vector<const State*> &states_;

            /** \brief When sampling near-by states, this number decides how many attempts are made to satisfy the desired distance */
            unsigned int                     maxNearSamplesAttempts_;

        private:
            std::size_t                      maxStateIndex_;
        };
    }
}
