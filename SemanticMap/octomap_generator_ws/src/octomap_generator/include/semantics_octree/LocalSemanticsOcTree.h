/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LOCAL_SEMANTICS_OCTREE_H
#define LOCAL_SEMANTICS_OCTREE_H

#include <semantics_octree/semantics_octree_node.h>

namespace octomap {

template <class SEMANTICS>
class LocalSemanticsOcTreeNode : public SemanticsOcTreeNode<SEMANTICS> {
public:
    LocalSemanticsOcTreeNode()
        : SemanticsOcTreeNode<SEMANTICS>()
        , time_stamp(0)
    {
    }
    
    LocalSemanticsOcTreeNode(const LocalSemanticsOcTreeNode& rhs)
        : SemanticsOcTreeNode<SEMANTICS>(rhs)
        , time_stamp(rhs.time_stamp)
    {
    }

    bool operator==(const LocalSemanticsOcTreeNode& rhs) const
    {
        //return (rhs.value == value && rhs.time_stamp == time_stamp);
        return (rhs.time_stamp == time_stamp);
    }

    void copyData(const LocalSemanticsOcTreeNode& from)
    {
        SemanticsOcTreeNode<SEMANTICS>::copyData(from);
        time_stamp = from.getTimestamp();
    }

    inline unsigned int GetTimestamp() const { return time_stamp; }

    inline void UpdateTimestamp() { time_stamp = (unsigned int)time(NULL); }

    inline void SetTimestamp(unsigned int t) { time_stamp = t; }

    inline void UpdateOccupancyChildren()
    {
        this->setLogOdds(this->getMaxChildLogOdds()); // conservative
        UpdateTimestamp();
    }

protected:
    unsigned int time_stamp;
};

#if 1
template <class SEMANTICS>
class LocalSemanticsOcTree : public SemanticsOcTree<LocalSemanticsOcTreeNode<SEMANTICS>> {
public:
    LocalSemanticsOcTree(double resolution) {}
    //LocalSemanticsOcTree* create() const { return new LocalSemanticsOcTree(resolution); }
    std::string getTreeType() const { return "LocalSemanticsOcTree"; }

    unsigned int GetLastUpdateTime();
    void DegradeOutdatedNodes(unsigned int time_thres);
    void IntegrateMissNoTime(LocalSemanticsOcTree* node) const;

    virtual void updateNodeLogOdds(LocalSemanticsOcTree* node, const float& update) const;

protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer {
    public:
        StaticMemberInitializer()
        {
            LocalSemanticsOcTree* tree = new LocalSemanticsOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

        /**
      * Dummy function to ensure that MSVC does not drop the
      * StaticMemberInitializer, causing this tree failing to register.
      * Needs to be called from the constructor of this octree.
      */
        void ensureLinking() {};
    };

    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeStampedMemberInit;
};
#endif

} // end namespace octomap

#endif // LOCAL_SEMANTICS_OCTREE_H