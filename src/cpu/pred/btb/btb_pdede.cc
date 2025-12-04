#include "btb_pdede.hh"

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
    pageBits(p.pageBits)
{
    /*
        Totally 8192 entries
        Monitor table: 2 Align Banks
            // Solution 1
            each bank has 4096 entries
            each entry has 4 ways, 1024 entries per way
            for way 0 to way 1, offset bits are 11, 11,
            for way 2 to way 3, offset bits are 11, with PagePointer

            // Solution 2
            each bank has 4096 entries
            each entry has 8 ways, 512 entries per way
            for way 0 to way 6, offset bits are 0, 4, 5, 7, 9, 11,
            for way 6 to way 7, offset bits are 11, with PagePointer

        Page table:
            2 align banks use the same page table, totally 512 entries
            each entry has 16 ways, 32 entries per way
            each entry in a way has pageBits bits of tag
            we use targetOffset bits [11:7] to index the page table
            if cross-page, we use Cat(pcHigher, pageTag, targetOffset, 0.B(instShiftAmt)) to form the full target
            otherwise, we use Cat(pcHigher, pagePointerWway, targetOffset, 0.B(instShiftAmt)) to form the full target

        We choose Solution 1 when numWays == 4, Solution 2 when numWays == 8
    */

    numAlignBanks = predictWidth / blockSize; // 2 align banks

    assert(numWays == 4 || numWays == 8);

    if (numWays == 4) {
        wayOffsetBits = way4OffsetBits;
        wayUsePagePointer = way4UsePagePointer;
    } else if (numWays == 8) {
        wayOffsetBits = way8OffsetBits;
        wayUsePagePointer = way8UsePagePointer;
    }

    // Initialize monitor table
    numSets = numEntries / (numWays * numAlignBanks);
    monitorTable.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        monitorTable[bank].resize(numSets);
        for (unsigned set = 0; set < numSets; ++set) {
            monitorTable[bank][set].reserve(numWays);
            for (unsigned way = 0; way < numWays; ++way) {
                monitorTable[bank][set].emplace_back(
                    wayOffsetBits[way], wayUsePagePointer[way]);
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
        pageTable[getPageTableIdx(targetLower)][entry.pagePointerWay];

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
            fullTarget = targetLower; // invalid target
        }
    }
    else if (entry.isUsePagePointer()) {
        // we use Cat(pcHigher, pagePointerWay, targetOffset, 0.B(instShiftAmt)) to form the full target
        pcMiddle = pc & ~mask(floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt);
        pcMiddlePlusOne = pcMiddle + (1ULL << (floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt));
        pcMiddleMinusOne = pcMiddle - (1ULL << (floorLog2(numPageWays) + entry.getOffsetBits() + instShiftAmt));

        Addr pageSection = entry.pagePointerWay << (maxOffsetBits + instShiftAmt);

        if (carry.isFit()) {
            fullTarget = pcMiddle | pageSection | targetLower;
        }
        else if (carry.isPlusOne()) {
            fullTarget = pcMiddlePlusOne | pageSection | targetLower;
        }
        else if (carry.isMinusOne()){
            fullTarget = pcMiddleMinusOne | pageSection | targetLower;
        }
        else {
            fullTarget = targetLower; // invalid target
        }
    }
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
    for (unsigned i = 0; i < tagBits; i += tagFoldedBits) {
        tagHigher ^= (fullTag & mask(tagFoldedBits));
        fullTag = fullTag >> tagFoldedBits;
    }

    if (tagFoldedBits == 0) assert(tagHigher == 0);

    return tagLower | (tagHigher << (tagBits - tagFoldedBits));
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

    meta->rawMonitorSets = res;

    return res;
}

Addr BTBPDede::getPageTableIdx(Addr pc) {
    // select high floorLog2(numPageSets) bits from targetOffset
    // example: floorLog2(numPageSets) = 5, maxOffsetBits = 11, instShiftAmt = 1
    // then we select bits [11:11-5+1] = bits [11:7] from targetOffset
    unsigned setWidth = floorLog2(numPageSets);
    Addr idx = (pc >> (maxOffsetBits - setWidth + instShiftAmt)) & (numPageSets - 1);
    return idx;
}

Addr BTBPDede::getPageTableTag(Addr pc) {
    Addr fullTag = pc >> (maxOffsetBits + instShiftAmt);
    return fullTag & mask(pageBits);
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
        for (unsigned way = 0; way < numWays; ++way) {
            auto &entry = bank[way];
            Addr branchPC = alignedAddr + (entry.position << instShiftAmt);
            if (!entry.valid) continue;
            if (entry.tag != getMonitorTag(alignedAddr)) continue;
            if (branchPC < pc || branchPC >= (pc + predictWidth)) continue;

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

            // update entry LRU state
            unsigned alignedBankIdx = getRotatedAlignBankIdx(alignedAddr, i);
            Addr monitorIdx = getMonitorIdx(alignedAddr);
            unsigned currentPLRUState = monitorPLRUTable[alignedBankIdx][monitorIdx];
            unsigned touchedPLRUState = getTouchedPLRUState(
                currentPLRUState,
                numWays,
                way
            );
            monitorPLRUTable[alignedBankIdx][monitorIdx] = touchedPLRUState;
            DPRINTF(BTBPDede, "BTBPDede: touched monitor PLRU state for bank %d idx %#lx from %#x to %#x\n",
                alignedBankIdx, monitorIdx, currentPLRUState, touchedPLRUState);

            // update page table LRU state if using page pointer
            if (entry.isUsePagePointer() && entry.isCrossPage) {
                Addr pageTableIdx = getPageTableIdx(alignedAddr);
                unsigned currentPagePLRUState = pagePLRUTable[pageTableIdx];
                unsigned touchedPagePLRUState = getTouchedPLRUState(
                    currentPagePLRUState,
                    numPageWays,
                    entry.pagePointerWay
                );
                pagePLRUTable[pageTableIdx] = touchedPagePLRUState;
                DPRINTF(BTBPDede, "BTBPDede: touched page table PLRU state for idx %#lx from %#x to %#x\n",
                    pageTableIdx, currentPagePLRUState, touchedPagePLRUState);
            }
        }
    }
    std::sort(btbEntries.begin(), btbEntries.end(),
        [](const BTBEntry &a, const BTBEntry &b) {
            return a.pc < b.pc;
        }
    );
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
}

void BTBPDede::putPCHistory(
    Addr startAddr,
    const boost::dynamic_bitset<> &history,
    std::vector<FullBTBPrediction> &stagePreds
)
{
    DPRINTF(BTBPDede, "===== BTBPDede: putPCHistory called for startAddr %#lx =====\n", startAddr);

    meta = std::make_shared<BTBPDedeMeta>();
    // Lookup monitor entries
    auto monitorEntries = getMonitorEntries(startAddr);

    auto processed_entries = processMonitorEntries(startAddr, monitorEntries);

    fillStagePredictions(processed_entries, stagePreds);
}

unsigned BTBPDede::getTargetDiffBits(Addr pc, Addr target) {
    Addr diff = (pc >> instShiftAmt) ^ (target >> instShiftAmt);
    unsigned diffBits = 0;
    while (diff != 0) {
        diffBits++;
        diff = diff >> 1;
    }
    return diffBits;
}

unsigned BTBPDede::getPartitionIdx(const BranchInfo &exec, const std::shared_ptr<BTBPDedeMeta> &meta) {
    Addr pc = exec.pc;
    Addr target = exec.target;
    bool isReturn = exec.isReturn;

    DPRINTF(BTBPDede, "BTBPDede: getPartitionIdx called for pc %#lx target %#lx isReturn %d\n",
        pc, target, isReturn);

    Addr alignedPC = pc & ~(blockSize - 1);
    unsigned alignBankIdx = getRotatedAlignBankIdx(pc, 0);

    auto getSmallestPartitionIdx = [this](unsigned diffBits) -> unsigned {
        for (unsigned i = 0; i < wayOffsetBits.size(); ++i) {
            if (diffBits <= wayOffsetBits[i]) {
                return i;
            }
            else if (wayUsePagePointer[i]) {
                return i;
            }
        }
        return wayOffsetBits.size() - 1;
    };

    unsigned diffBits = // if is return, we can allocate it to any partition
        !isReturn ? getTargetDiffBits(pc, target) : 0;
    unsigned startIdx = getSmallestPartitionIdx(diffBits);
    unsigned finalIdx = startIdx;
    bool foundEmpty = false;
    for (unsigned i = startIdx; i < wayOffsetBits.size(); ++i) {
        // choose the first way that is empty
        if (!meta->rawMonitorSets[alignBankIdx][i].valid) {
            finalIdx = i;
            foundEmpty = true;
            break;
        }
    }
    DPRINTF(BTBPDede, "BTBPDede: getPartitionIdx computed diffBits %d startIdx %d\n",
        diffBits, startIdx);
    /* if !foundEmpty, we use PLRU to choose the way;
     * NOTE: in this case, PLRU will give numWay victims,
     *       so we need to filter the ways that are smaller than startIdx,
     *       and choose the first one among the rest.
     */
    if (foundEmpty) {
        DPRINTF(BTBPDede, "BTBPDede: chosen partitionIdx %d (empty) for pc %#lx\n",
            finalIdx, pc);
        return finalIdx;
    }

    auto monitorIdx = getMonitorIdx(alignedPC);
    auto plruState = monitorPLRUTable[alignBankIdx][monitorIdx];
    auto plruVictims = getPLRUVictims(plruState, numWays);
    for (auto way : plruVictims) {
        if (way >= startIdx) {
            finalIdx = way;
            break;
        }
    }
    for (auto way : plruVictims) {
        DPRINTF(BTBPDede, "BTBPDede: PLRU victim way %d\n", way);
    }
    DPRINTF(BTBPDede, "BTBPDede: chosen partitionIdx %d (PLRU) for pc %#lx\n",
        finalIdx, pc);

    return finalIdx;
}

void BTBPDede::update(const FetchStream& stream) {
    static uint64_t lastUpdateTick;

    DPRINTF(BTBPDede, "===== BTBPDede: update called for exePC %#lx =====\n", stream.exeBranchInfo.pc);

    if (stream.squashType != SQUASH_CTRL) {
        DPRINTF(BTBPDede, "BTBPDede: update skipped due to non-control squash\n");
        return;
    }
    if (!stream.exeTaken) {
        DPRINTF(BTBPDede, "BTBPDede: update skipped due to not taken branch\n");
        return;
    }

    if (lastUpdateTick == stream.predTick) {
        DPRINTF(BTBPDede, "BTBPDede: update skipped due to already updated in this prediction tick\n");
        return;
    }

    lastUpdateTick = stream.predTick;

    auto metaFromUpdate =
        std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);

    BranchInfo exec = stream.exeBranchInfo;
    unsigned alignedBankIdx = getRotatedAlignBankIdx(exec.pc, 0);
    unsigned monitorIdx = getMonitorIdx(exec.pc);
    unsigned alignedPosition = (exec.pc & (blockSize - 1)) >> instShiftAmt;

    MonitorSet toUpdate = metaFromUpdate->rawMonitorSets[alignedBankIdx];

    BranchAttribute execAttr({
        exec.isCond ? BranchAttribute::BranchTypeEnum::Conditional :
            (exec.isIndirect ? BranchAttribute::BranchTypeEnum::Indirect :
                BranchAttribute::BranchTypeEnum::Direct),
        exec.isReturn ? BranchAttribute::RasActionEnum::Pop :
            (exec.isCall ? BranchAttribute::RasActionEnum::Push :
                BranchAttribute::RasActionEnum::None)
    });


    // TODO: is really need this check?
    // bool mispredHitMeta = false;
    // for (auto &entry : metaFromUpdate->rawMonitorSets[alignedBankIdx]) {
    //     if (!entry.valid) continue; // skip invalid entries
    //     if (!(entry.attr.branchType == execAttr.branchType &&
    //           entry.attr.rasAction == execAttr.rasAction)) continue; // skip different attributes
    //     if (entry.position != alignedPosition) continue; // skip different position
    //     mispredHitMeta = true;
    //     break;
    // }

    // if mispredHitMeta, we can infer that BTB is not responsible for the misprediction
    // if (mispredHitMeta) return;

    unsigned targetDiffBits = getTargetDiffBits(exec.pc, exec.target);
    unsigned partitionIdx = getPartitionIdx(exec, metaFromUpdate);
    bool usePagePointer = wayUsePagePointer[partitionIdx];

    // find entry already hit in monitor table
    unsigned foundWay = -1;
    for (unsigned way = 0; way < numWays; ++way) {
        auto &entry = toUpdate[way];
        if (!entry.valid) continue; // skip invalid entries
        if (entry.position != alignedPosition) continue; // skip different position
        if (entry.tag != getMonitorTag(exec.pc)) continue; // skip different tag
        foundWay = way;
        break;
    }

    if (foundWay != -1 && foundWay != partitionIdx) {
        DPRINTF(BTBPDede, "BTBPDede: found existing monitor entry in way %d, need to invalid\n",
            foundWay);
        // invalidate the found entry
        toUpdate[foundWay].valid = false;
        // update PLRU state
        // foundWay should be replaced as soon as possible
        // so we set it as the most recently used entry
        auto currentPLRUState = monitorPLRUTable[alignedBankIdx][monitorIdx];
        auto newPLRUState = getMakeVictimPLRUState(
            currentPLRUState,
            numWays,
            foundWay
        );
        monitorPLRUTable[alignedBankIdx][monitorIdx] = newPLRUState;
        DPRINTF(BTBPDede, "BTBPDede: updated monitor PLRU state for bank %d idx %#lx from %#x to %#x\n",
            alignedBankIdx, monitorIdx, currentPLRUState, newPLRUState);
    }

    MonitorEntry &monitorEntry = toUpdate[partitionIdx];

    // Update monitor entry
    monitorEntry.valid = true;
    monitorEntry.position = alignedPosition;
    monitorEntry.tag = getMonitorTag(exec.pc);
    monitorEntry.targetOffset = (exec.target >> instShiftAmt) & mask(monitorEntry.getOffsetBits());
    if (usePagePointer && (targetDiffBits > maxOffsetBits + floorLog2(numPageWays))) {
        DPRINTF(BTBPDede, "BTBPDede: using page pointer for monitor entry update\n");

        Addr pagePointerSet = getPageTableIdx(exec.pc);
        // check if page entry exists
        Addr pagePointerWay = 0;
        bool pageEntryExists = false;
        for (unsigned way = 0; way < numPageWays; ++way) {
            auto &pageEntry = pageTable[pagePointerSet][way];
            Addr newTag = getPageTableTag(exec.pc);
            Addr entryTag = pageEntry.tag;
            if (entryTag == newTag) {
                pagePointerWay = way;
                pageEntryExists = true;
                break;
            }
        }

        monitorEntry.isCrossPage = true;
        if (pageEntryExists) {
            DPRINTF(BTBPDede, "BTBPDede: page entry exists in way %d\n", pagePointerWay);
            monitorEntry.pagePointerWay = pagePointerWay;
        }
        else {
            DPRINTF(BTBPDede, "BTBPDede: page entry does not exist, allocating new entry\n");
            // choose victim way using PLRU
            auto plruState = pagePLRUTable[pagePointerSet];
            auto plruVictims = getPLRUVictims(plruState, numPageWays);
            pagePointerWay = plruVictims[0]; // choose the first victim
            monitorEntry.pagePointerWay = pagePointerWay;

            // update page entry
            DPRINTF(BTBPDede, "BTBPDede: updating page entry at set %d, way %d\n",
                pagePointerSet, pagePointerWay);
            auto &pageEntry = pageTable[pagePointerSet][pagePointerWay];
            pageEntry.tag = getPageTableTag(exec.pc);
            DPRINTF(BTBPDede, "BTBPDede: updated page entry details:\n");
            printPageEntry(pageEntry);

            // update monitor entry carry bits
            monitorEntry.carry = computeCarryBits(
                exec.pc,
                exec.target,
                maxOffsetBits + pageBits
            );
        }

        // update page PLRU state
        unsigned currentPLRUState = pagePLRUTable[pagePointerSet];
        unsigned touchedPLRUState = getTouchedPLRUState(
            currentPLRUState,
            numPageWays,
            pagePointerWay
        );
        pagePLRUTable[pagePointerSet] = touchedPLRUState;
        DPRINTF(BTBPDede, "BTBPDede: touched page table PLRU state for idx %#lx from %#x to %#x\n",
            pagePointerSet, currentPLRUState, touchedPLRUState);
    }
    else if (usePagePointer && (targetDiffBits <= maxOffsetBits + floorLog2(numPageWays))) {
        DPRINTF(BTBPDede, "BTBPDede: using page pointer for monitor entry update (not cross page)\n");

        monitorEntry.isCrossPage = false;
        monitorEntry.pagePointerWay = (exec.target >> (maxOffsetBits + instShiftAmt)) & mask(floorLog2(numPageWays));

        monitorEntry.carry = computeCarryBits(
            exec.pc,
            exec.target,
            maxOffsetBits + floorLog2(numPageWays)
        );
        monitorEntry.attr = execAttr;
    }
    else {
        DPRINTF(BTBPDede, "BTBPDede: not using page pointer for monitor entry update\n");
        monitorEntry.carry = computeCarryBits(
            exec.pc,
            exec.target,
            monitorEntry.getOffsetBits()
        );
        monitorEntry.attr = execAttr;
    }

    DPRINTF(BTBPDede, "BTBPDede: updated monitor entry at bank %d, index %d, way %d\n",
        alignedBankIdx, monitorIdx, partitionIdx);

    DPRINTF(BTBPDede, "BTBPDede: updated monitor entry details:\n");
    printMonitorEntry(monitorEntry);

    monitorTable[alignedBankIdx][monitorIdx] = toUpdate;

    // update monitor PLRU state
    auto currentPLRUState = monitorPLRUTable[alignedBankIdx][monitorIdx];
    auto newPLRUState = getTouchedPLRUState(
        currentPLRUState,
        numWays,
        partitionIdx
    );
     monitorPLRUTable[alignedBankIdx][monitorIdx] = newPLRUState;
    DPRINTF(BTBPDede, "BTBPDede: updated monitor PLRU state for bank %d idx %#lx from %#x to %#x\n",
        alignedBankIdx, monitorIdx, currentPLRUState, newPLRUState);
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
        "pagePointerWay:%#lx, carry:%d, attr:(branchType:%d, rasAction:%d)\n",
        e.getOffsetBits(), e.isUsePagePointer(), e.valid, e.isCrossPage,
        e.position, e.tag, e.targetOffset, getPageTableIdx(e.targetOffset << instShiftAmt),
        e.pagePointerWay, (int)e.carry.targetCarry,
        (int)e.attr.branchType, (int)e.attr.rasAction);
}

void BTBPDede::printPageEntry(const PageEntry& e) {
    DPRINTF(BTBPDede, "PageEntry: tag:%#lx, ctr:%d\n",
        e.tag, e.ctr);
}

}
}
}
