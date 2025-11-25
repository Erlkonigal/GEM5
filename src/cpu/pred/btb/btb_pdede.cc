#include "btb_pdede.hh"

namespace gem5 {
namespace branch_prediction {
namespace btb_pred {

BTBPDEDE::BTBPDEDE(const Params& p):
    TimedBaseBTBPredictor(p),
    instShiftAmt(p.instShiftAmt),
    numEntries(p.numEntries),
    numWays(p.numWays),
    tagBits(p.tagBits),
    pageBits(p.pageBits)
{
    /*
        Totally 8192 entries
        Monitor table: 2 Align Banks
            // Solution 1
            each bank has 4096 entries
            each entry has 4 ways, 1024 entries per way
            for way 0 to way 2, offset bits are 0, 4, 7
            for way 3, offset bits are 11, with PagePointer, optionally RegionPointer

            // Solution 2
            each bank has 4096 entries
            each entry has 8 ways, 512 entries per way
            for way 0 to way 6, offset bits are 0, 4, 5, 7, 9, 11,
            for way 6 to way 7, offset bits are 11, with PagePointer

        We choose Solution 1 when numWays == 4, Solution 2 when numWays == 8
    */

    numAlignBanks = 2;

    // TODO: implement Solution 2
    assert(numWays == 4);

    // TODO: offset bits need profiling to decide
    const std::vector<unsigned> wayOffsetBits = {0, 4, 7, 11};
    const std::vector<bool> wayUsesPagePointer = {false, false, false, true};
    // Initialize monitor table
    unsigned numSets = numEntries / (numWays * numAlignBanks); // 2 banks
    monitorTable.resize(numSets);
    for (auto& set : monitorTable) {
        set.reserve(numWays);
        for (unsigned way = 0; way < numWays; ++way) {
            unsigned offsetBits = wayOffsetBits[way];
            set.emplace_back(offsetBits);
        }
    }

}


}
}
}
