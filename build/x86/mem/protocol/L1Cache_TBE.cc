/** \file L1Cache_TBE.cc
 *
 * Auto generated C++ code started by /mnt/hgfs/ShareShen/gem5-origin-stable-2015-9-3/src/mem/slicc/symbols/Type.py:410
 */

#include <iostream>
#include <memory>

#include "mem/protocol/L1Cache_TBE.hh"
#include "mem/ruby/common/Global.hh"
#include "mem/ruby/system/System.hh"

using namespace std;
/** \brief Print the state of this object */
void
L1Cache_TBE::print(ostream& out) const
{
    out << "[L1Cache_TBE: ";
    out << "TBEState = " << m_TBEState << " ";
    out << "DataBlk = " << m_DataBlk << " ";
    out << "]";
}
