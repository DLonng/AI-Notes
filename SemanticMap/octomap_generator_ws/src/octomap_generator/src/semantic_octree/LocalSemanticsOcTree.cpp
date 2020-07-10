#include "semantics_octree/LocalSemanticsOcTree.h"

namespace octomap {

template <class SEMANTICS>
LocalSemanticsOcTree<SEMANTICS>::LocalSemanticsOcTree(double resolution)
    : SemanticsOcTree<LocalSemanticsOcTreeNode<SEMANTICS>>(resolution)
{
    ocTreeStampedMemberInit.ensureLinking();
}

template <class SEMANTICS>
unsigned int LocalSemanticsOcTree<SEMANTICS>::GetLastUpdateTime() {
    return root->getTimestamp();
}





} // namespace octoma
