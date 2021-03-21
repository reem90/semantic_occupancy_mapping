/*
 * Copyright 2015 Andreas Bircher, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <semantic_occupancy_mapping_3d/rrt_tree.h>


semMAP::MapBase::MapBase()
{
    counter_ = 0;
}

semMAP::MapBase::MapBase(OctomapGeneratorBase *octomap_generator_)
{
    manager_ = octomap_generator_;
    counter_ = 0;
}

semMAP::MapBase::~MapBase()
{
}

void semMAP::MapBase::setParams(Params params)
{
    params_ = params;
}

int semMAP::MapBase::getCounter()
{
    return counter_;
}

