#include "btb_pdede.hh"
#include "cpu/o3/dyn_inst.hh"

namespace gem5 {
namespace branch_prediction {
namespace btb_pred {

BTBPDede::BTBPDede(const Params& p):
    TimedBaseBTBPredictor(p),
    instShiftAmt(p.instShiftAmt),
    numEntries(p.numEntries),
    numPageEntries(p.numPageEntries),
    numRegionEntries(p.numRegionEntries),
    numWays(p.numWays),
    numPageWays(p.numPageWays),
    numRegionWays(p.numRegionWays),
    tagBits(p.tagBits),
    tagFoldedBits(p.tagFoldedBits),
    pageBits(p.pageBits),
    victimCacheEntries(p.victimCacheEntries),
    stats(this)
{
    /*
        Totally 8192 entries
        Monitor table: 2 Align Banks

            each bank has 4096 entries
            each entry has numWay ways, 4096/numWay entries per way
            for way 0 to way numWay-1, offset bits are 11, with PagePointer

        Page table:
            2 align banks use the same page table, totally 512 entries
            each entry has 16 ways, 32 entries per way
            each entry in a way has pageBits bits of tag
            we use targetOffset bits [11:7] to index the page table
            if cross-page, we use Cat(pcHigher, pageTag, targetOffset, 0.B(instShiftAmt)) to form the full target
            otherwise, we use Cat(pcHigher, pagePointerWay, targetOffset, 0.B(instShiftAmt)) to form the full target

        We choose Solution 1 when numWays == 4, Solution 2 when numWays == 8
    */

    numAlignBanks = predictWidth / blockSize; // 2 align banks

    // Initialize monitor table
    numSets = numEntries / (numWays * numAlignBanks);
    monitorTable.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        monitorTable[bank].resize(numSets);
        for (unsigned set = 0; set < numSets; ++set) {
            monitorTable[bank][set].reserve(numWays);
            for (unsigned way = 0; way < numWays; ++way) {
                monitorTable[bank][set].emplace_back();
            }
        }
    }
    // Initialize page table
    numPageSets = numPageEntries / numPageWays;
    pageTable.resize(numPageSets);
    for (unsigned set = 0; set < numPageSets; ++set) {
        pageTable[set].resize(numPageWays);
        for (unsigned way = 0; way < numPageWays; ++way) {
            pageTable[set][way].tag = 0;
            pageTable[set][way].ctr = 0;
        }
    }

    // Initialize monitor plru table
    monitorPLRUTable.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        monitorPLRUTable[bank].resize(numSets);
        for (unsigned set = 0; set < numSets; ++set) {
            monitorPLRUTable[bank][set] = 0;
        }
    }

    // Initialize page plru table
    pagePLRUTable.resize(numPageSets);
    for (unsigned set = 0; set < numPageSets; ++set) {
        pagePLRUTable[set] = 0;
    }

    // Initialize victim cache
    numVictimCacheSets = victimCacheEntries / numAlignBanks;
    victimCache.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        victimCache[bank].reserve(numVictimCacheSets);
        for (unsigned set = 0; set < numVictimCacheSets; ++set) {
            victimCache[bank].emplace_back(20, false);
        }
    }

    victimCachePLRUTable.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        victimCachePLRUTable[bank] = 0;
    }


    DPRINTF(BTBPDede, "BTBPDede initialized: numEntries %d, numWays %d, numSets %d, "
        "tagBits %d, tagFoldedBits %d, pageBits %d, numPageEntries %d, "
        "numPageWays %d, numPageSets %d\n",
        numEntries, numWays, numSets, tagBits, tagFoldedBits, pageBits,
        numPageEntries, numPageWays, numPageSets);
}

BTBPDede::~BTBPDede()
{

}

std::shared_ptr<void> BTBPDede::getPredictionMeta()
{
    return meta;
}

unsigned BTBPDede::getRotatedAlignBankIdx(Addr pc, unsigned logicBankIdx)
{
    // Rotate align bank index based on PC bits to reduce conflicts
    unsigned alignBankWidth = floorLog2(blockSize);
    unsigned rotation = (pc >> alignBankWidth) & (numAlignBanks - 1);
    return (logicBankIdx + rotation) % numAlignBanks;
}

Addr BTBPDede::getFullTarget(Addr pc, const MonitorEntry &entry)
{
    Addr targetLower = entry.targetOffset << instShiftAmt;

    const auto &pageEntry =
        pageTable[entry.pageTableSet][entry.extendedInfo.pageTableWay];

    Addr fullTarget = 0;
    TargetCarry carry = entry.carry;

    Addr pcUpper = pc & ~mask(pageBits + maxOffsetBits + instShiftAmt);
    Addr pcUpperPlusOne = pcUpper + (1ULL << (pageBits + maxOffsetBits + instShiftAmt));
    Addr pcUpperMinusOne = pcUpper - (1ULL << (pageBits + maxOffsetBits + instShiftAmt));

    Addr pcMiddle = pc & ~mask(entry.getOffsetBits() + instShiftAmt);
    Addr pcMiddlePlusOne = pcMiddle + (1ULL << (entry.getOffsetBits() + instShiftAmt));
    Addr pcMiddleMinusOne = pcMiddle - (1ULL << (entry.getOffsetBits() + instShiftAmt));

    if (entry.isUsePagePointer() && entry.isCrossPage) {
        Addr pageSection = pageEntry.tag << (maxOffsetBits + instShiftAmt);

        if (carry.isFit()) {
            fullTarget = pcUpper | pageSection | targetLower;
        }
        else if (carry.isPlusOne()) {
            fullTarget = pcUpperPlusOne | pageSection | targetLower;
        }
        else if (carry.isMinusOne()){
            fullTarget = pcUpperMinusOne | pageSection | targetLower;
        }
        else {
            fullTarget = pageSection | targetLower; // invalid target
        }
    }
    // else if (entry.isUsePagePointer()) {
    //     // we use Cat(pcHigher, pagePointerWay, targetOffset, 0.B(instShiftAmt)) to form the full target
    //     pcMiddle = pc & ~mask(floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt);
    //     pcMiddlePlusOne = pcMiddle + (1ULL << (floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt));
    //     pcMiddleMinusOne = pcMiddle - (1ULL << (floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt));

    //     Addr pageSection = entry.pagePointerWay << (maxOffsetBits + instShiftAmt);

    //     if (carry.isFit()) {
    //         fullTarget = pcMiddle | pageSection | targetLower;
    //     }
    //     else if (carry.isPlusOne()) {
    //         fullTarget = pcMiddlePlusOne | pageSection | targetLower;
    //     }
    //     else if (carry.isMinusOne()){
    //         fullTarget = pcMiddleMinusOne | pageSection | targetLower;
    //     }
    //     else {
    //         fullTarget = pageSection | targetLower; // invalid target
    //     }
    // }
    else {
        if (carry.isFit()) {
            fullTarget = pcMiddle | targetLower;
        }
        else if (carry.isPlusOne()) {
            fullTarget = pcMiddlePlusOne | targetLower;
        }
        else if (carry.isMinusOne()){
            fullTarget = pcMiddleMinusOne | targetLower;
        }
        else {
            fullTarget = targetLower; // invalid target
        }
    }

    return fullTarget;
}

BTBPDede::TargetCarry BTBPDede::computeCarryBits(Addr pc, Addr target, unsigned offsetBits)
{
    TargetCarry carry;
    Addr pcUpper = pc & ~mask(offsetBits + instShiftAmt);
    Addr pcUpperPlusOne = pcUpper + (1ULL << (offsetBits + instShiftAmt));
    Addr pcUpperMinusOne = pcUpper - (1ULL << (offsetBits + instShiftAmt));

    Addr targetLower = target & mask(offsetBits + instShiftAmt);

    Addr candidateFit = pcUpper | targetLower;
    Addr candidatePlusOne = pcUpperPlusOne | targetLower;
    Addr candidateMinusOne = pcUpperMinusOne | targetLower;

    if (candidateFit == target) {
        carry.targetCarry = TargetCarry::TargetCarryEnum::Fit;
    }
    else if (candidatePlusOne == target) {
        carry.targetCarry = TargetCarry::TargetCarryEnum::PlusOne;
    }
    else if (candidateMinusOne == target) {
        carry.targetCarry = TargetCarry::TargetCarryEnum::MinusOne;
    }
    else {
        carry.targetCarry = TargetCarry::TargetCarryEnum::None;
    }

    stats.carryOverflowTimes += carry.isNone();
    return carry;
}

/*                bits storage example for 8-way PLRU binary tree:
 *                      bit[6]: ways 7-4 older than ways 3-0
 *                      /                                  \
 *            bit[5]: ways 7+6 > 5+4                bit[2]: ways 3+2 > 1+0
 *            /                    \                /                    \
 *     bit[4]: way 7>6    bit[3]: way 5>4    bit[1]: way 3>2    bit[0]: way 1>0
 */
std::vector<unsigned> BTBPDede::getPLRUVictims(unsigned state, unsigned numWays)
{
    assert(isPowerOf2(numWays));

    std::vector<unsigned> ways;

    if (numWays > 2) {
        unsigned rightWays = numWays / 2;
        unsigned leftWays = numWays - rightWays;
        unsigned leftSubtreeOlder = (state >> (numWays - 2)) & 0x1;
        unsigned leftSubtreeState = (state >> (rightWays - 1)) & mask(leftWays - 1);
        unsigned rightSubtreeState = state & mask(rightWays - 1);

        assert(leftWays == rightWays); // numWays is power of 2

        auto leftVictims = getPLRUVictims(leftSubtreeState, leftWays);
        auto rightVictims = getPLRUVictims(rightSubtreeState, rightWays);

        if (leftSubtreeOlder) {
            for (auto w : leftVictims) {
                ways.push_back(w | (1 << floorLog2(leftWays)));
            }
            for (auto w : rightVictims) {
                ways.push_back(w & ~(1 << floorLog2(rightWays)));
            }
        }
        else {
            for (auto w : rightVictims) {
                ways.push_back(w & ~(1 << floorLog2(rightWays)));
            }
            for (auto w : leftVictims) {
                ways.push_back(w | (1 << floorLog2(leftWays)));
            }
        }
    }
    else if (numWays == 2) {
        ways.push_back(state & 0x1);
        ways.push_back(!(state & 0x1));
    }
    else {
        assert(false); // should not reach here
    }

    return ways;
}

unsigned BTBPDede::getTouchedPLRUState(unsigned state, unsigned numWays, unsigned touchWay) {
    assert(isPowerOf2(numWays));

    unsigned touchedState = 0;

    if (numWays > 2) {
        unsigned rightWays = numWays / 2;
        unsigned leftWays = numWays - rightWays;
        unsigned setLeftOlder = !((touchWay >> (floorLog2(numWays) - 1)) & 0x1);
        unsigned leftSubtreeState = (state >> (rightWays - 1)) & mask(leftWays - 1);
        unsigned rightSubtreeState = state & mask(rightWays - 1);

        if (setLeftOlder) {
            unsigned touchedRightState = getTouchedPLRUState(
                rightSubtreeState,
                rightWays,
                touchWay & mask(floorLog2(rightWays))
            );
            touchedState |= (1 << (numWays - 2)); // set left older bit
            touchedState |= leftSubtreeState << (rightWays - 1);
            touchedState |= touchedRightState;
        }
        else {
            unsigned touchedLeftState = getTouchedPLRUState(
                leftSubtreeState,
                leftWays,
                touchWay & mask(floorLog2(leftWays))
            );
            touchedState &= ~(1 << (numWays - 2)); // clear left older bit
            touchedState |= touchedLeftState << (rightWays - 1);
            touchedState |= rightSubtreeState;
        }

    }
    else if (numWays == 2) {
        touchedState = !(touchWay & 0x1);
    }
    else {
        assert(false); // should not reach here
    }

    assert(touchedState < (1 << (numWays - 1)));

    return touchedState;
}

unsigned BTBPDede::getMakeVictimPLRUState(unsigned state, unsigned numWays, unsigned victimWay) {
    assert(isPowerOf2(numWays));

    unsigned victimizedState = 0;

    if (numWays > 2) {
        unsigned rightWays = numWays / 2;
        unsigned leftWays = numWays - rightWays;
        unsigned setLeftOlder = (victimWay >> (floorLog2(numWays) - 1)) & 0x1;
        unsigned leftSubtreeState = (state >> (rightWays - 1)) & mask(leftWays - 1);
        unsigned rightSubtreeState = state & mask(rightWays - 1);

        if (setLeftOlder) {
            unsigned victimizedLeftState = getMakeVictimPLRUState(
                leftSubtreeState,
                leftWays,
                victimWay & mask(floorLog2(leftWays))
            );
            victimizedState |= (1 << (numWays - 2)); // set left older bit
            victimizedState |= victimizedLeftState << (rightWays - 1);
            victimizedState |= rightSubtreeState;
        }
        else {
            unsigned victimizedRightState = getMakeVictimPLRUState(
                rightSubtreeState,
                rightWays,
                victimWay & mask(floorLog2(rightWays))
            );
            victimizedState &= ~(1 << (numWays - 2)); // clear left older bit
            victimizedState |= leftSubtreeState << (rightWays - 1);
            victimizedState |= victimizedRightState;
        }
    }
    else if (numWays == 2) {
        victimizedState = (victimWay & 0x1);
    }
    else {
        assert(false); // should not reach here
    }

    assert(victimizedState < (1 << (numWays - 1)));

    return victimizedState;
}

Addr BTBPDede::getMonitorIdx(Addr pc)
{
    unsigned fetchBlockWidth = floorLog2(predictWidth);
    Addr idx = (pc >> fetchBlockWidth) & (numSets - 1);
    return idx;
}

Addr BTBPDede::getMonitorTag(Addr pc)
{
    unsigned fetchBlockWidth = floorLog2(predictWidth);
    unsigned setWidth = floorLog2(numSets);
    Addr fullTag = pc >> (fetchBlockWidth + setWidth);
    // use folded xor for higher tagFoldedBits bits
    Addr tagHigher = 0;
    Addr tagLower = fullTag & mask(tagBits - tagFoldedBits);

    // if no folded bits, return directly
    if (tagFoldedBits == 0) return tagLower;

    // fold higher bits
    fullTag = fullTag >> (tagBits - tagFoldedBits);
    for (unsigned i = 0; i < tagBits; i += tagFoldedBits) {
        tagHigher ^= (fullTag & mask(tagFoldedBits));
        fullTag = fullTag >> tagFoldedBits;
    }
    tagHigher = tagHigher << (tagBits - tagFoldedBits);

    // lower and higher should not overlap
    assert((tagLower & tagHigher) == 0);

    return tagLower | tagHigher;
}

std::vector<BTBPDede::MonitorSet> BTBPDede::getMonitorEntries(Addr pc)
{
    std::vector<MonitorSet> res(numAlignBanks);

    unsigned alignBankWidth = floorLog2(blockSize);
    Addr alignedStartAddr = pc & ~(blockSize - 1);

    for (unsigned i = 0; i < numAlignBanks; ++i) {
        unsigned phyBankIdx = getRotatedAlignBankIdx(pc, i);
        Addr alignedAddr = alignedStartAddr + blockSize * i;
        Addr idx = getMonitorIdx(alignedAddr);
        MonitorSet monitorSet = monitorTable[phyBankIdx][idx];
        res[phyBankIdx] = monitorSet;
    }

    return res;
}

Addr BTBPDede::getPageTableIdx(Addr target) {
    // Solution 1
    // select high floorLog2(numPageSets) bits from targetOffset
    // example: floorLog2(numPageSets) = 5, maxOffsetBits = 11, instShiftAmt = 1
    // then we select bits [11:11-5+1] = bits [11:7] from targetOffset
    // unsigned setWidth = floorLog2(numPageSets);
    // Addr idx = (target >> (maxOffsetBits - setWidth + instShiftAmt)) & (numPageSets - 1);

    // Solution 2
    // select low floorLog2(numPageSets) bits from targetOffset
    // example: floorLog2(numPageSets) = 5, maxOffsetBits = 11, instShiftAmt = 1
    // then we select bits [5:1] from targetOffset
    // Addr idx = (target >> instShiftAmt) & (numPageSets - 1);

    unsigned setWidth = floorLog2(numPageSets);
    Addr idxFull = (target >> (instShiftAmt + maxOffsetBits));

    // use idxFull[setWidth * 2 - 1:setWidth] ^ idxFull[setWidth - 1:0] to reduce conflicts
    Addr idxHigher = (idxFull >> setWidth) & mask(setWidth);
    Addr idxLower = idxFull & mask(setWidth);
    Addr idx = idxHigher ^ idxLower;

    return idx;
}

Addr BTBPDede::getPageTableTag(Addr target) {
    Addr fullTag = target >> (maxOffsetBits + instShiftAmt);
    return fullTag & mask(pageBits);
}

Addr BTBPDede::getVictimCacheTag(Addr monitorTag, Addr monitorIdx)
{
    unsigned monitorSetWidth = floorLog2(numSets);
    Addr victimTag = (monitorTag << monitorSetWidth) | monitorIdx;
    return victimTag;
}

std::vector<BTBEntry> BTBPDede::processMonitorEntries(Addr pc, const std::vector<MonitorSet> &originEntries)
{
    auto monitorEntries = originEntries;
    std::vector<BTBEntry> btbEntries;

    // collect all valid entries
    for (unsigned i = 0; i < numAlignBanks; ++i) {
        unsigned phyBankIdx = getRotatedAlignBankIdx(pc, i);
        auto &bank = monitorEntries[phyBankIdx];
        Addr alignedAddr = (pc & ~(blockSize - 1)) + blockSize * i;
        bool foundInVictimCache = false;

        for (unsigned way = 0; way < numWays + numVictimCacheSets; ++way) {
            MonitorEntry &entry =
                (way < numWays) ? bank[way] : victimCache[phyBankIdx][way - numWays];

            Addr branchPC = alignedAddr + (entry.position << instShiftAmt);
            Addr monitorIdx = getMonitorIdx(alignedAddr);
            Addr monitorTag = getMonitorTag(alignedAddr);
            Addr victimTag = getVictimCacheTag(monitorTag, monitorIdx);

            if (!entry.valid) continue;
            if (branchPC < pc || branchPC >= (pc + predictWidth)) continue;
            if (entry.tag != (way < numWays ? monitorTag : victimTag)) continue;

            DPRINTF(BTBPDede, "BTBPDede: use entry from %s way %d for bank %d alignedAddr %#lx\n",
                (way < numWays) ? "monitor" : "victim cache",
                (way < numWays) ? way : (way - numWays),
                phyBankIdx,
                alignedAddr);

            if (way >= numWays) {
                foundInVictimCache = true;
                stats.predHitVictimEntries++;
            }

            BTBEntry btbEntry;
            btbEntry.valid = true;
            btbEntry.pc = branchPC;
            btbEntry.tag = entry.tag;
            btbEntry.target = getFullTarget(branchPC, entry);
            btbEntry.isCond = (entry.attr.branchType == BranchAttribute::BranchTypeEnum::Conditional);
            btbEntry.isIndirect = (entry.attr.branchType == BranchAttribute::BranchTypeEnum::Indirect);
            btbEntry.isCall = (entry.attr.rasAction == BranchAttribute::RasActionEnum::Push);
            btbEntry.isReturn = (entry.attr.rasAction == BranchAttribute::RasActionEnum::Pop);
            btbEntry.size = 1 << instShiftAmt; // assume size is 2^instShiftAmt
            btbEntries.push_back(btbEntry);
            DPRINTF(BTBPDede, "BTBPDede: found valid BTB entry pc %#lx target %#lx\n",
                btbEntry.pc, btbEntry.target);
            DPRINTF(BTBPDede, "BTBPDede: entry details - isCond %d, isIndirect %d, isCall %d, isReturn %d\n",
                btbEntry.isCond, btbEntry.isIndirect, btbEntry.isCall, btbEntry.isReturn);

            if (way >= numWays) {
                // update victim cache PLRU state
                unsigned currentVictimPLRUState = victimCachePLRUTable[phyBankIdx];
                unsigned touchedVictimPLRUState = getTouchedPLRUState(
                    currentVictimPLRUState,
                    numVictimCacheSets,
                    way - numWays
                );
                victimCachePLRUTable[phyBankIdx] = touchedVictimPLRUState;
                DPRINTF(BTBPDede, "BTBPDede: touched victim cache PLRU state for bank %d from %#x to %#x\n",
                    phyBankIdx, currentVictimPLRUState, touchedVictimPLRUState);
                continue;
            }

            // update entry PLRU state
            unsigned currentPLRUState = monitorPLRUTable[phyBankIdx][monitorIdx];
            unsigned touchedPLRUState = getTouchedPLRUState(
                currentPLRUState,
                numWays,
                way
            );
            monitorPLRUTable[phyBankIdx][monitorIdx] = touchedPLRUState;
            DPRINTF(BTBPDede, "BTBPDede: touched monitor PLRU state for bank %d idx %#lx from %#x to %#x\n",
                phyBankIdx, monitorIdx, currentPLRUState, touchedPLRUState);

            // update page table LRU state if using page pointer
            if (entry.isUsePagePointer() && entry.isCrossPage) {
                Addr pageTableIdx = entry.pageTableSet;
                unsigned currentPagePLRUState = pagePLRUTable[pageTableIdx];
                unsigned touchedPagePLRUState = getTouchedPLRUState(
                    currentPagePLRUState,
                    numPageWays,
                    entry.extendedInfo.pageTableWay
                );
                pagePLRUTable[pageTableIdx] = touchedPagePLRUState;
                DPRINTF(BTBPDede, "BTBPDede: touched page table PLRU state for idx %#lx from %#x to %#x\n",
                    pageTableIdx, currentPagePLRUState, touchedPagePLRUState);
            }
        }

        if (foundInVictimCache) {
            stats.predHitVictimTimes++;
        }
    }


    std::sort(btbEntries.begin(), btbEntries.end(),
        [](const BTBEntry &a, const BTBEntry &b) {
            return a.pc < b.pc;
        }
    );

    // // prevent duplicate entries with same PC
    // // this should happen only when victim cache and monitor table
    // // both have entries for the same branch
    // auto last = std::unique(btbEntries.begin(), btbEntries.end(),
    //     [](const BTBEntry &a, const BTBEntry &b) {
    //         return a.pc == b.pc;
    //     }
    // );
    // if (last != btbEntries.end()) {
    //     DPRINTF(BTBPDede, "BTBPDede: removed %d duplicate BTB entries with same PC\n",
    //         std::distance(last, btbEntries.end()));
    // }
    // btbEntries.erase(last, btbEntries.end());

    if (btbEntries.size()) stats.predHitTimes++;
    else stats.predMissTimes++;

    stats.predHitEntries += btbEntries.size();

    BTBPDedeMeta newMeta;
    newMeta.rawMonitorSets = originEntries;
    newMeta.btbEntries = btbEntries;

    meta = std::make_shared<BTBPDedeMeta>(newMeta);

    return btbEntries;
}

void BTBPDede::fillStagePredictions(
    const std::vector<BTBEntry>& btbEntries,
    std::vector<FullBTBPrediction>& stagePreds
)
{
    auto checkAscending = [](std::vector<BTBEntry> &es) {
        Addr last = 0;
        bool misorder = false;
        for (auto &entry : es) {
            if (entry.pc <= last) {
                misorder = true;
                break;
            }
            last = entry.pc;
        }
        if (misorder) {
            fatal("BTBPDede: BTB entries are not in ascending order of PC!");
        }
    };

    FillStageLoop(s) {
        DPRINTF(BTBPDede, "BTBPDede: assigning prediction for stage %d\n", s);
        // Copy BTB entries to stage prediction
        stagePreds[s].btbEntries.clear();
        for (auto e : btbEntries) {
            stagePreds[s].btbEntries.push_back(e);
        }
        checkAscending(stagePreds[s].btbEntries);
        if (s == getDelay()) dumpBTBEntries(stagePreds[s].btbEntries);

        stagePreds[s].predTick = curTick();

        stagePreds[s].condTakens.clear();
        stagePreds[s].indirectTargets.clear();
    }

        // Set predictions for each branch
    for (auto &e : btbEntries) {
        assert(e.valid);
        if (e.isCond) {
            // TODO: a performance bug here, mbtb should not update condTakens!
            // if (isL0()) {  // only L0 BTB has saturating counter
            // use saturating counter of L0 BTB

            // FillStageLoop(s) stagePreds[s].condTakens.push_back({e.pc, e.alwaysTaken || (e.ctr >= 0)});

            // } else {  // L1 BTB condTakens depends on the TAGE predictor
            // }
        } else if (e.isIndirect) {
            // Set predicted target for indirect branches
            DPRINTF(BTBPDede, "setting indirect target for pc %#lx to %#lx\n", e.pc, e.target);

            FillStageLoop(s) stagePreds[s].indirectTargets.push_back({e.pc, e.target});

            if (e.isReturn) {
                FillStageLoop(s) stagePreds[s].returnTarget = e.target;
            }
            break;
        }
    }
}

void BTBPDede::putPCHistory(
    Addr startAddr,
    const boost::dynamic_bitset<> &history,
    std::vector<FullBTBPrediction> &stagePreds
)
{
    DPRINTF(BTBPDede, "===== BTBPDede: putPCHistory called for startAddr %#lx =====\n", startAddr);
    stats.predTimes++;

    // Lookup monitor entries
    auto monitorEntries = getMonitorEntries(startAddr);

    auto processed_entries = processMonitorEntries(startAddr, monitorEntries);

    fillStagePredictions(processed_entries, stagePreds);
}

void BTBPDede::update(const FetchStream& stream) {
    DPRINTF(BTBPDede, "===== BTBPDede: update called for exePC %#lx =====\n", stream.exeBranchInfo.pc);

    if (stream.squashType != SQUASH_CTRL) {
        DPRINTF(BTBPDede, "BTBPDede: update skipped due to non-control squash\n");
        return;
    }
    if (!stream.exeTaken) {
        DPRINTF(BTBPDede, "BTBPDede: update skipped due to not taken branch\n");
        return;
    }

    stats.updateTimes++;

    auto metaFromUpdate =
        std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);

    BranchInfo exec = stream.exeBranchInfo;
    unsigned alignedBankIdx = getRotatedAlignBankIdx(exec.pc, 0);
    unsigned monitorIdx = getMonitorIdx(exec.pc);
    unsigned monitorTag = getMonitorTag(exec.pc);
    unsigned pageTableIdx = getPageTableIdx(exec.target);
    unsigned pageTableTag = getPageTableTag(exec.target);
    unsigned alignedPosition = (exec.pc & (blockSize - 1)) >> instShiftAmt;

    auto &toUpdate = monitorTable[alignedBankIdx][monitorIdx];

    BranchAttribute execAttr({
        exec.isCond ? BranchAttribute::BranchTypeEnum::Conditional :
            (exec.isIndirect ? BranchAttribute::BranchTypeEnum::Indirect :
                BranchAttribute::BranchTypeEnum::Direct),
        exec.isReturn ? BranchAttribute::RasActionEnum::Pop :
            (exec.isCall ? BranchAttribute::RasActionEnum::Push :
                BranchAttribute::RasActionEnum::None)
    });

    // find entry already hit in monitor table
    unsigned foundWay = -1;
    for (unsigned way = 0; way < numWays; ++way) {
        auto &entry = toUpdate[way];
        if (!entry.valid) continue; // skip invalid entries
        if (entry.position != alignedPosition) continue; // skip different position
        if (entry.tag != monitorTag) continue; // skip different tag

        foundWay = way;
        stats.updateHitTimes++;
        break;
    }

    // find entry already hit in victim cache
    unsigned foundVictimWay = -1;
    for (unsigned way = 0; way < numVictimCacheSets; ++way) {
        auto &entry = victimCache[alignedBankIdx][way];
        Addr victimTag = getVictimCacheTag(monitorTag, monitorIdx);

        if (!entry.valid) continue; // skip invalid entries
        if (entry.position != alignedPosition) continue; // skip different position
        if (entry.tag != victimTag) continue; // skip different tag

        foundVictimWay = way;
        stats.updateHitVictimTimes++;
        break;
    }

    // helper functions for update
    auto updateMonitorPLRUState = [&] (unsigned way) {
        unsigned currentPLRUState = monitorPLRUTable[alignedBankIdx][monitorIdx];
        unsigned touchedPLRUState = getTouchedPLRUState(
            currentPLRUState,
            numWays,
            way
        );
        monitorPLRUTable[alignedBankIdx][monitorIdx] = touchedPLRUState;
        DPRINTF(BTBPDede, "BTBPDede: touched monitor PLRU state for bank %d idx %#lx from %#x to %#x\n",
            alignedBankIdx, monitorIdx, currentPLRUState, touchedPLRUState);
    };

    auto updatePagePLRUState = [&](unsigned way) {
        unsigned currentPLRUState = pagePLRUTable[pageTableIdx];
        unsigned touchedPLRUState = getTouchedPLRUState(
            currentPLRUState,
            numPageWays,
            way
        );
        pagePLRUTable[pageTableIdx] = touchedPLRUState;
        DPRINTF(BTBPDede, "BTBPDede: touched page table PLRU state for idx %#lx from %#x to %#x\n",
            pageTableIdx, currentPLRUState, touchedPLRUState);
    };

    auto updateVictimCachePLRUState = [&](unsigned way) {
        unsigned currentVictimPLRUState = victimCachePLRUTable[alignedBankIdx];
        unsigned touchedVictimPLRUState = getTouchedPLRUState(
            currentVictimPLRUState,
            numVictimCacheSets,
            way
        );
        victimCachePLRUTable[alignedBankIdx] = touchedVictimPLRUState;
        DPRINTF(BTBPDede, "BTBPDede: touched victim cache PLRU state for bank %d from %#x to %#x\n",
            alignedBankIdx, currentVictimPLRUState, touchedVictimPLRUState);
    };

    // build new entry
    TargetCarry shortCarry = computeCarryBits(
        exec.pc,
        exec.target,
        maxOffsetBits
    );
    TargetCarry longCarry = computeCarryBits(
        exec.pc,
        exec.target,
        maxOffsetBits + pageBits
    );

    bool isCrossPage = shortCarry.isNone();

    MonitorEntry newEntry;
    newEntry.valid = true;
    newEntry.isCrossPage = isCrossPage;
    newEntry.position = alignedPosition;
    newEntry.tag = monitorTag;
    newEntry.targetOffset = (exec.target >> instShiftAmt) & mask(maxOffsetBits);
    newEntry.attr = execAttr;
    newEntry.carry = (isCrossPage) ? longCarry : shortCarry;
    newEntry.pageTableSet = pageTableIdx;

    // compute extended info
    if (isCrossPage) {
        stats.updateUsePagePointerTimes++;

        // check if page entry exists
        Addr pageTableWay = -1;
        for (unsigned way = 0; way < numPageWays; ++way) {
            auto &pageEntry = pageTable[pageTableIdx][way];
            Addr entryTag = pageEntry.tag;
            if (entryTag != pageTableTag) continue;

            pageTableWay = way;
            break;
        }
        // page entry not exists
        if (pageTableWay == -1) {
            stats.updateAllocatePagePointerTimes++;

            // choose victim way using PLRU
            pageTableWay = getPLRUVictims(
                pagePLRUTable[pageTableIdx],
                numPageWays
            )[0];

            // update page entry
            auto &pageEntry = pageTable[pageTableIdx][pageTableWay];
            pageEntry.tag = getPageTableTag(exec.target);

            DPRINTF(BTBPDede, "BTBPDede: updating page entry at set %d, way %d\n",
                pageTableIdx, pageTableWay);
            DPRINTF(BTBPDede, "BTBPDede: updated page entry details:\n");
            printPageEntry(pageEntry);
        }

        newEntry.extendedInfo.pageTableWay = pageTableWay;
        updatePagePLRUState(pageTableWay);
    }
    else {
        // TODO: add saturated counter update logic
    }

    if (foundWay != -1) {
        toUpdate[foundWay] = newEntry;

        DPRINTF(BTBPDede, "BTBPDede: updated existing monitor entry at bank %d, index %d, way %d\n",
            alignedBankIdx, monitorIdx, foundWay);

        updateMonitorPLRUState(foundWay);

        // deduplicate victim cache if foundVictimWay also hits
        if (foundVictimWay != -1) {
            DPRINTF(BTBPDede, "BTBPDede: deduplicating victim cache entry at bank %d, way %d\n",
                alignedBankIdx, foundVictimWay);
            victimCache[alignedBankIdx][foundVictimWay].valid = false;
        }
    }
    else if (foundVictimWay != -1) {
        // in-place update existing victim cache entry
        auto &victimEntry = victimCache[alignedBankIdx][foundVictimWay];
        victimEntry = newEntry;
        victimEntry.tag = getVictimCacheTag(monitorTag, monitorIdx);

        updateVictimCachePLRUState(foundVictimWay);

        DPRINTF(BTBPDede, "BTBPDede: updated existing victim cache entry at bank %d, way %d\n",
            alignedBankIdx, foundVictimWay);
    }
    else {
        // need to allocate new entry
        stats.updateMissTimes++;

        unsigned evictWay = -1;

        // firstly try to find invalid entry
        for (unsigned i = 0; i < numWays; ++i) {
            auto &entry = toUpdate[i];
            if (!entry.valid) {
                evictWay = i;
                DPRINTF(BTBPDede, "BTBPDede: found empty way %d for bank %d idx %#lx\n",
                    evictWay, alignedBankIdx, monitorIdx);
                break;
            }
        }

        if (evictWay == -1) {
            stats.updateEvictTimes++;
            // no invalid entry, use PLRU to choose victim
            evictWay = getPLRUVictims(
                monitorPLRUTable[alignedBankIdx][monitorIdx],
                numWays
            )[0];

            updateMonitorPLRUState(evictWay);

            DPRINTF(BTBPDede, "BTBPDede: evicting way %d for bank %d idx %#lx\n",
                evictWay, alignedBankIdx, monitorIdx);
        }

        const auto &toEvictEntry = toUpdate[evictWay];

        // evict to victim cache
        if (toEvictEntry.valid && victimCacheEntries != 0) {
            // insert evicted entry into victim cache
            unsigned victimWay = -1;
            for (unsigned way = 0; way < numVictimCacheSets; way++) {
                auto &victimEntry = victimCache[alignedBankIdx][way];
                // find invalid entry first
                if (!victimEntry.valid) {
                    victimWay = way;
                    DPRINTF(BTBPDede, "BTBPDede: found empty victim cache way %d for bank %d\n",
                        victimWay, alignedBankIdx);
                    break;
                }
                if (victimEntry.tag != getVictimCacheTag(toEvictEntry.tag, monitorIdx)) continue;
                if (victimEntry.position != toEvictEntry.position) continue;
                // update existing entry in victim cache
                victimWay = way;
                break;
            }

            if (victimWay == -1) {
                // no invalid entry, use victim cache PLRU to choose victim
                victimWay = getPLRUVictims(
                    victimCachePLRUTable[alignedBankIdx],
                    numVictimCacheSets
                )[0];
                DPRINTF(BTBPDede, "BTBPDede: evicting victim cache way %d for bank %d\n",
                    victimWay, alignedBankIdx);

                updateVictimCachePLRUState(victimWay);
            }

            auto &victimEntry = victimCache[alignedBankIdx][victimWay];
            victimEntry = toEvictEntry;
            victimEntry.tag = getVictimCacheTag(toEvictEntry.tag, monitorIdx);
        }

        toUpdate[evictWay] = newEntry;
    }
}

void BTBPDede::printBTBEntry(const BTBEntry& e) {
    DPRINTF(BTBPDede, "BTBEntry: valid %d, pc:%#lx, tag: %#lx, size:%d, target:%#lx, "
        "cond:%d, indirect:%d, call:%d, return:%d, always_taken:%d\n",
        e.valid, e.pc, e.tag, e.size, e.target, e.isCond, e.isIndirect,
        e.isCall, e.isReturn, e.alwaysTaken);

}

void BTBPDede::dumpBTBEntries(const std::vector<BTBEntry>& es) {
    DPRINTF(BTBPDede, "BTBEntries:\n");
    for (const auto &entry : es) {
        printBTBEntry(entry);
    }
}

void BTBPDede::printMonitorEntry(const MonitorEntry& e) {
    DPRINTF(BTBPDede, "MonitorEntry: offsetBits:%d, usePagePointer:%d, valid:%d, isCrossPage:%d, "
        "position:%d, tag:%#lx, targetOffset:%#lx, pagePointerSet:%#lx, "
        "extInfo:%#lx, carry:%d, attr:(branchType:%d, rasAction:%d)\n",
        e.getOffsetBits(), e.isUsePagePointer(), e.valid, e.isCrossPage,
        e.position, e.tag, e.targetOffset, e.pageTableSet,
        e.extendedInfo.pageTableWay, (int)e.carry.targetCarry,
        (int)e.attr.branchType, (int)e.attr.rasAction);
}

void BTBPDede::printPageEntry(const PageEntry& e) {
    DPRINTF(BTBPDede, "PageEntry: tag:%#lx, ctr:%d\n",
        e.tag, e.ctr);
}

void BTBPDede::commitBranch(const FetchStream &stream, const DynInstPtr &inst) {
    auto meta = std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);
    const auto &rawEntries = meta->rawMonitorSets;
    const auto &btbEntries = meta->btbEntries;

    auto pc = inst->getPC();
    auto npc = inst->getNPC();
    bool branchHit = false;
    auto entry = BTBEntry();
    for (const auto &e : btbEntries) {
        if (e.pc == pc) {
            branchHit = true;
            entry = e;
            break;
        }
    }

    bool condNotTaken = inst->isCondCtrl() && !inst->branching();
    bool hitBranchTaken = stream.exeTaken && stream.getControlPC() == pc;

    // unsigned targetDiffBits = getTargetDiffBits(pc, npc);

    stats.totalBranchHits += branchHit;
    stats.totalBranchMisses += !branchHit;

    if (branchHit) {
        stats.totalBranchHits++;
        if (hitBranchTaken) {
            // stats.totalBranchHitTakens++;
        } else {
            // stats.totalBranchHitNotTakens++;
        }
        if (inst->isCondCtrl()) {
            stats.condHits++;
            // stats.condTargetDiffBits.sample(targetDiffBits);
            if (hitBranchTaken) {
                // stats.condHitTakens++;
            } else {
                // stats.condHitNotTakens++;
            }

            bool pred_taken = entry.ctr >= 0;
            if (pred_taken == hitBranchTaken) {
                // stats.condPredCorrect++;
            } else {
                // stats.condPredWrong++;
            }

        }
        if (inst->isUncondCtrl()) {
            stats.uncondHits++;
            // stats.uncondTargetDiffBits.sample(targetDiffBits);
        }
        // ignore non-speculative branches (e.g. syscall)
        if (!inst->isNonSpeculative()) {
            if (inst->isIndirectCtrl()) {
                stats.indirectHits++;
                // stats.indirectTargetDiffBits.sample(targetDiffBits);
                Addr pred_target = entry.target;
                if (pred_target == npc) {
                    // stats.indirectPredCorrect++;
                } else {
                    // stats.indirectPredWrong++;
                }
            }
            if (inst->isCall()) {
                stats.callHits++;
                // stats.callTargetDiffBits.sample(targetDiffBits);
            }
            if (inst->isReturn()) {
                stats.returnHits++;
                // stats.returnTargetDiffBits.sample(targetDiffBits);
            }
        }
    } else {
        stats.totalBranchMisses++;
        if (hitBranchTaken) {
            // stats.totalBranchMissTakens++;
        } else {
            // stats.totalBranchMissNotTakens++;
        }
        if (inst->isCondCtrl()) {
            stats.condMisses++;
            if (hitBranchTaken) {
                // stats.condMissTakens++;
                // stats.condPredWrong++;

            } else {
                // stats.condMissNotTakens++;
                // stats.condPredCorrect++;
            }
        }
        if (inst->isUncondCtrl()) {
            stats.uncondMisses++;
        }
        // ignore non-speculative branches (e.g. syscall)
        if (!inst->isNonSpeculative()) {
            if (inst->isIndirectCtrl()) {
                stats.indirectMisses++;
                // stats.indirectPredWrong++;
            }
            if (inst->isCall()) {
                stats.callMisses++;
            }
            if (inst->isReturn()) {
                stats.returnMisses++;
            }
        }
    }
}

BTBPDede::PDedeStats::PDedeStats(statistics::Group *parent) :
    statistics::Group(parent, "BTBPDede"),
    ADD_STAT(predTimes, statistics::units::Count::get(), "Number of predictions made"),
    ADD_STAT(predMissTimes, statistics::units::Count::get(), "Number of prediction misses"),
    ADD_STAT(predHitTimes, statistics::units::Count::get(), "Number of prediction hits"),
    ADD_STAT(predHitEntries, statistics::units::Count::get(), "Number of predicted entries hit"),
    ADD_STAT(predHitVictimTimes, statistics::units::Count::get(),
        "Number of predicted times from victim cache"),
    ADD_STAT(predHitVictimEntries, statistics::units::Count::get(),
        "Number of predicted entries from victim cache"),
    ADD_STAT(updateTimes, statistics::units::Count::get(), "Number of updates made"),
    ADD_STAT(updateMissTimes, statistics::units::Count::get(), "Number of update misses"),
    ADD_STAT(updateFoundEmptyTimes, statistics::units::Count::get(),"Number of update where an empty entry was found"),
    ADD_STAT(updateEvictTimes, statistics::units::Count::get(),"Number of update where an entry was evicted"),
    ADD_STAT(updateHitTimes, statistics::units::Count::get(), "Number of update hits"),
    ADD_STAT(updateHitVictimTimes, statistics::units::Count::get(),
        "Number of update hits from victim cache"),
    ADD_STAT(updateMultiHitTimes, statistics::units::Count::get(), "Number of update multi-hits"),
    ADD_STAT(updateUsePagePointerTimes, statistics::units::Count::get(),
        "Number of updates using page pointer"),
    ADD_STAT(updateAllocatePagePointerTimes, statistics::units::Count::get(),
        "Number of updates allocating new page pointer"),
    ADD_STAT(updateNotUseButHasPagePointerTimes, statistics::units::Count::get(),
        "Number of updates not using page pointer but having page pointer"),
    ADD_STAT(updateNotUseAndNoPagePointerTimes, statistics::units::Count::get(),
        "Number of updates not using page pointer and no page pointer"),
    ADD_STAT(carryOverflowTimes, statistics::units::Count::get(),
        "Number of times carry overflow occurred during updates"),
    ADD_STAT(totalBranchHits, statistics::units::Count::get(), "Total number of branch hits in BTB"),
    ADD_STAT(totalBranchMisses, statistics::units::Count::get(), "Total number of branch misses in BTB"),
    ADD_STAT(condHits, statistics::units::Count::get(), "Number of conditional branch hits"),
    ADD_STAT(condMisses, statistics::units::Count::get(), "Number of conditional branch misses"),
    ADD_STAT(uncondHits, statistics::units::Count::get(), "Number of unconditional branch hits"),
    ADD_STAT(uncondMisses, statistics::units::Count::get(), "Number of unconditional branch misses"),
    ADD_STAT(indirectHits, statistics::units::Count::get(), "Number of indirect branch hits"),
    ADD_STAT(indirectMisses, statistics::units::Count::get(), "Number of indirect branch misses"),
    ADD_STAT(callHits, statistics::units::Count::get(), "Number of call branch hits"),
    ADD_STAT(callMisses, statistics::units::Count::get(), "Number of call branch misses"),
    ADD_STAT(returnHits, statistics::units::Count::get(), "Number of return branch hits"),
    ADD_STAT(returnMisses, statistics::units::Count::get(), "Number of return branch misses")
    // ADD_STAT(condTargetDiffBits, statistics::units::Count::get(),
    //     "Target difference bits for conditional branch updates"),
    // ADD_STAT(uncondTargetDiffBits, statistics::units::Count::get(),
    //     "Target difference bits for unconditional branonds"),
    // ADD_STAT(indirectTargetDiffBits, statistics::units::Count::get(),
    //     "Target difference bits for indirect branch updates"),
    // ADD_STAT(callTargetDiffBits, statistics::units::Count::get(),
    //     "Target difference bits for call branch updates"),
    // ADD_STAT(returnTargetDiffBits, statistics::units::Count::get(),
    //     "Target difference bits for return branch updates"),
    // ADD_STAT(condAllocPartitionIdx, statistics::units::Count::get(),
    //     "Partition index allocated for conditional branch updates"),
    // ADD_STAT(uncondAllocPartitionIdx, statistics::units::Count::get(),
    //     "Partition index allocated for unconditional branch updates"),
    // ADD_STAT(indirectAllocPartitionIdx, statistics::units::Count::get(),
    //     "Partition index allocated for indirect branch updates"),
    // ADD_STAT(callAllocPartitionIdx, statistics::units::Count::get(),
    //     "Partition index allocated for call branch updates"),
    // ADD_STAT(returnAllocPartitionIdx, statistics::units::Count::get(),
    //     "Partition index allocated for return branch updates")
{
}


}
}
}
