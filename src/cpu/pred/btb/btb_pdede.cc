#include <algorithm>

#include "btb_pdede.hh"
#include "cpu/o3/dyn_inst.hh"

namespace gem5 {
namespace branch_prediction {
namespace btb_pred {

namespace {

constexpr uint8_t kMaxRRPV = 3;

uint8_t
srripTouch()
{
    return 0;
}

template <class SetType>
unsigned
findOrSelectSRRIPWay(SetType &set)
{
    if (set.empty()) {
        return 0;
    }

    for (unsigned way = 0; way < set.size(); ++way) {
        if (!set[way].valid) {
            return way;
        }
    }

    while (true) {
        for (unsigned way = 0; way < set.size(); ++way) {
            if (set[way].rrpv == kMaxRRPV) {
                return way;
            }
        }
        for (auto &entry : set) {
            if (entry.rrpv < kMaxRRPV) {
                entry.rrpv++;
            }
        }
    }
}

} // anonymous namespace

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
            pageTable[set][way].valid = false;
            pageTable[set][way].tag = 0;
            pageTable[set][way].regionWay = 0;
            pageTable[set][way].rrpv = maxRRPV;
        }
    }

    // Initialize region table
    numRegionSets = numRegionEntries / numRegionWays;
    assert(numRegionSets == 1);
    regionTable.resize(numRegionSets);
    for (unsigned set = 0; set < numRegionSets; ++set) {
        regionTable[set].resize(numRegionWays);
        for (unsigned way = 0; way < numRegionWays; ++way) {
            regionTable[set][way].valid = false;
            regionTable[set][way].tag = 0;
            regionTable[set][way].rrpv = maxRRPV;
        }
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


    DPRINTF(BTBPDede, "BTBPDede initialized: numEntries %d, numWays %d, numSets %d, "
        "tagBits %d, tagFoldedBits %d, pageBits %d, numPageEntries %d, "
        "numPageWays %d, numPageSets %d, numRegionEntries %d, numRegionWays %d, "
        "numRegionSets %d\n",
        numEntries, numWays, numSets, tagBits, tagFoldedBits, pageBits,
        numPageEntries, numPageWays, numPageSets,
        numRegionEntries, numRegionWays, numRegionSets);
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

    Addr fullTarget = 0;
    TargetCarry carry = entry.targetCarry;

    Addr pcMiddle = pc & ~mask(entry.getOffsetBits() + instShiftAmt);
    Addr pcMiddlePlusOne = pcMiddle + (1ULL << (entry.getOffsetBits() + instShiftAmt));
    Addr pcMiddleMinusOne = pcMiddle - (1ULL << (entry.getOffsetBits() + instShiftAmt));

    // For long targets, reconstruct with regionTag + pageTag + targetLower.
    if (entry.isUsePagePointer() && entry.isCrossPage) {
        if (entry.pageTableSet >= numPageSets || entry.pageTableWay >= numPageWays) {
            return targetLower;
        }
        const auto &pageEntry = pageTable[entry.pageTableSet][entry.pageTableWay];
        if (!pageEntry.valid || pageEntry.regionWay >= numRegionWays) {
            return targetLower;
        }
        constexpr unsigned regionSet = 0;
        const auto &regionEntry = regionTable[regionSet][pageEntry.regionWay];
        if (!regionEntry.valid) {
            return targetLower;
        }

        Addr pageSection = pageEntry.tag << (maxOffsetBits + instShiftAmt);
        Addr regionSection =
            regionEntry.tag << (pageBits + maxOffsetBits + instShiftAmt);
        fullTarget = regionSection | pageSection | targetLower;
    }
    else {
        // Non-crossPage entries: use carry bits with pcMiddle
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

Addr BTBPDede::getRegionTableTag(Addr target) {
    return target >> (pageBits + maxOffsetBits + instShiftAmt);
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

            Addr branchPC;
            if (entry.isRVC) {
                branchPC = alignedAddr + (entry.position << 1);
            } else {
                branchPC = alignedAddr + (entry.position << 1) - 2;
            }
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
            btbEntry.size = entry.isRVC ? 2 : 4;
            // Set counter for conditional branches (only valid for non-crossPage entries)
            if (btbEntry.isCond && entry.isCrossPage) {
                printf("BTBPDede: found conditional branch entry with cross-page target"
                                ", which is not supported. pc %#lx\n",
                    btbEntry.pc);
                printf("BTBPDede: entry details - isCond %d, isIndirect %d, isCall %d, isReturn %d, isCrossPage %d\n",
                    btbEntry.isCond, btbEntry.isIndirect, btbEntry.isCall, btbEntry.isReturn, entry.isCrossPage);
                assert(!(btbEntry.isCond && entry.isCrossPage));
            }
            if (btbEntry.isCond && !entry.isCrossPage) {
                btbEntry.alwaysTaken = entry.alwaysTaken;
                btbEntry.ctr = entry.ctr;
            }
            btbEntries.push_back(btbEntry);
            DPRINTF(BTBPDede, "BTBPDede: found valid BTB entry pc %#lx target %#lx\n",
                btbEntry.pc, btbEntry.target);
            DPRINTF(BTBPDede, "BTBPDede: entry details - isCond %d, isIndirect %d, isCall %d, isReturn %d\n",
                btbEntry.isCond, btbEntry.isIndirect, btbEntry.isCall, btbEntry.isReturn);

            if (way >= numWays) {
                victimCache[phyBankIdx][way - numWays].rrpv = srripTouch();
                continue;
            }

            monitorTable[phyBankIdx][monitorIdx][way].rrpv = srripTouch();

            if (entry.isUsePagePointer() && entry.isCrossPage) {
                Addr pageTableIdx = entry.pageTableSet;
                if (pageTableIdx < numPageSets && entry.pageTableWay < numPageWays) {
                    auto &pageEntry = pageTable[pageTableIdx][entry.pageTableWay];
                    pageEntry.rrpv = srripTouch();
                    constexpr unsigned regionSet = 0;
                    if (pageEntry.regionWay < numRegionWays) {
                        regionTable[regionSet][pageEntry.regionWay].rrpv = srripTouch();
                    }
                }
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
            FillStageLoop(s) stagePreds[s].condTakens.push_back({e.pc, e.alwaysTaken || (e.ctr >= 0)});
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

std::vector<BTBEntry>
BTBPDede::prepareUpdateEntries(const FetchTarget &stream)
{
    auto all_entries = stream.updateBTBEntries;

    if (!stream.updateIsOldEntry) {
        BTBEntry potential_new_entry = stream.updateNewBTBEntry;
        bool new_entry_taken =
            stream.exeTaken && stream.getControlPC() == potential_new_entry.pc;
        if (!new_entry_taken) {
            potential_new_entry.alwaysTaken = false;
        }
        all_entries.push_back(potential_new_entry);
    }

    if (getResolvedUpdate()) {
        auto remove_it = std::remove_if(
            all_entries.begin(),
            all_entries.end(),
            [](const BTBEntry &e) { return !e.resolved; });
        all_entries.erase(remove_it, all_entries.end());
    }

    return all_entries;
}

void
BTBPDede::checkPredictionHit(const FetchTarget &stream,
                             const BTBPDede::BTBPDedeMeta *meta)
{
    bool pred_branch_hit = false;
    for (const auto &e : meta->btbEntries) {
        if (stream.exeBranchInfo == e) {
            pred_branch_hit = true;
            break;
        }
    }

    if (!pred_branch_hit && stream.exeTaken) {
        stats.updateMiss++;
    } else {
        stats.updateHit++;
    }
}

void BTBPDede::update(const FetchTarget& stream) {
    DPRINTF(BTBPDede, "===== BTBPDede: update called for exePC %#lx =====\n", stream.exeBranchInfo.pc);

    stats.updateTimes++;

    auto meta_from_update =
        std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);
    checkPredictionHit(stream, meta_from_update.get());

    auto entries_need_update = prepareUpdateEntries(stream);
    for (const auto &entry_to_update : entries_need_update) {
        BranchInfo exec = entry_to_update;
        unsigned alignedBankIdx = getRotatedAlignBankIdx(exec.pc, 0);
        unsigned monitorIdx = getMonitorIdx(exec.pc);
        unsigned monitorTag = getMonitorTag(exec.pc);
        unsigned pageTableIdx = getPageTableIdx(exec.target);
        unsigned pageTableTag = getPageTableTag(exec.target);

        bool isRVC = (exec.size == 2);
        unsigned offset = exec.pc & (blockSize - 1);
        unsigned alignedPosition = isRVC ? (offset >> 1) : ((offset + 2) >> 1);

        auto &toUpdate = monitorTable[alignedBankIdx][monitorIdx];

        BranchAttribute execAttr({
            exec.isCond ? BranchAttribute::BranchTypeEnum::Conditional :
                (exec.isIndirect ? BranchAttribute::BranchTypeEnum::Indirect :
                    BranchAttribute::BranchTypeEnum::Direct),
            exec.isReturn ? BranchAttribute::RasActionEnum::Pop :
                (exec.isCall ? BranchAttribute::RasActionEnum::Push :
                    BranchAttribute::RasActionEnum::None)
        });

        unsigned foundWay = numWays;
        for (unsigned way = 0; way < numWays; ++way) {
            auto &entry = toUpdate[way];
            if (!entry.valid) continue;
            if (entry.position != alignedPosition) continue;
            if (entry.tag != monitorTag) continue;
            foundWay = way;
            stats.updateHitTimes++;
            break;
        }

        unsigned foundVictimWay = numVictimCacheSets;
        for (unsigned way = 0; way < numVictimCacheSets; ++way) {
            auto &entry = victimCache[alignedBankIdx][way];
            Addr victimTag = getVictimCacheTag(monitorTag, monitorIdx);

            if (!entry.valid) continue;
            if (entry.position != alignedPosition) continue;
            if (entry.tag != victimTag) continue;

            foundVictimWay = way;
            stats.updateHitVictimTimes++;
            break;
        }
        if (foundWay != numWays && foundVictimWay != numVictimCacheSets) {
            stats.updateMultiHitTimes++;
        }

        TargetCarry shortCarry = computeCarryBits(exec.pc, exec.target, maxOffsetBits);
        bool isCrossPage = shortCarry.isNone();

        MonitorEntry newEntry;
        newEntry.valid = true;
        newEntry.isCrossPage = isCrossPage;
        newEntry.alwaysTaken = true;
        newEntry.isRVC = isRVC;
        newEntry.position = alignedPosition;
        newEntry.tag = monitorTag;
        newEntry.targetOffset = (exec.target >> instShiftAmt) & mask(maxOffsetBits);
        newEntry.attr = execAttr;
        newEntry.targetCarry = shortCarry;
        newEntry.rrpv = insertRRPV;
        newEntry.pageTableSet = pageTableIdx;

        if (isCrossPage) {
            stats.updateUsePagePointerTimes++;
            newEntry.targetCarry.targetCarry = TargetCarry::TargetCarryEnum::None;

            Addr regionTableTag = getRegionTableTag(exec.target);

            constexpr unsigned regionSet = 0;
            unsigned regionTableWay = numRegionWays;
            bool regionHit = false;
            for (unsigned way = 0; way < numRegionWays; ++way) {
                auto &regionEntry = regionTable[regionSet][way];
                if (!regionEntry.valid) continue;
                if (regionEntry.tag != regionTableTag) continue;

                regionTableWay = way;
                regionHit = true;
                break;
            }

            if (regionTableWay == numRegionWays) {
                regionTableWay = findOrSelectSRRIPWay(regionTable[regionSet]);
            }

            auto &regionEntry = regionTable[regionSet][regionTableWay];
            regionEntry.valid = true;
            regionEntry.tag = regionTableTag;
            regionEntry.rrpv = regionHit ? srripTouch() : insertRRPV;

            unsigned pageTableWay = numPageWays;
            bool pageHit = false;
            for (unsigned way = 0; way < numPageWays; ++way) {
                auto &pageEntry = pageTable[pageTableIdx][way];
                if (!pageEntry.valid) continue;
                Addr entryTag = pageEntry.tag;
                if (entryTag != pageTableTag) continue;

                pageTableWay = way;
                pageHit = true;
                break;
            }
            if (pageTableWay == numPageWays) {
                stats.updateAllocatePagePointerTimes++;

                pageTableWay = findOrSelectSRRIPWay(pageTable[pageTableIdx]);

                auto &pageEntry = pageTable[pageTableIdx][pageTableWay];
                pageEntry.valid = true;
                pageEntry.tag = getPageTableTag(exec.target);

                DPRINTF(BTBPDede, "BTBPDede: updating page entry at set %d, way %d\n",
                    pageTableIdx, pageTableWay);
                DPRINTF(BTBPDede, "BTBPDede: updated page entry details:\n");
                printPageEntry(pageEntry);
            }

            auto &pageEntry = pageTable[pageTableIdx][pageTableWay];
            pageEntry.valid = true;
            pageEntry.tag = getPageTableTag(exec.target);
            pageEntry.regionWay = regionTableWay;
            pageEntry.rrpv = pageHit ? srripTouch() : insertRRPV;

            newEntry.pageTableWay = pageTableWay;
        } else {
            newEntry.ctr = 0;
        }

        if (!isCrossPage &&
            execAttr.branchType == BranchAttribute::BranchTypeEnum::Conditional) {
            if (foundWay != numWays) {
                newEntry.ctr = toUpdate[foundWay].ctr;
                newEntry.alwaysTaken = toUpdate[foundWay].alwaysTaken;
            } else if (foundVictimWay != numVictimCacheSets) {
                newEntry.ctr = victimCache[alignedBankIdx][foundVictimWay].ctr;
                newEntry.alwaysTaken =
                    victimCache[alignedBankIdx][foundVictimWay].alwaysTaken;
            }

            bool this_cond_taken =
                stream.exeTaken && stream.getControlPC() == exec.pc;
            if (!this_cond_taken) {
                newEntry.alwaysTaken = false;
            }
            if (!newEntry.alwaysTaken) {
                if (this_cond_taken && newEntry.ctr < 1) {
                    newEntry.ctr++;
                }
                if (!this_cond_taken && newEntry.ctr > -2) {
                    newEntry.ctr--;
                }
            }
        }

        stats.updateTotal++;
        if (foundWay != numWays) {
            if (toUpdate[foundWay].targetOffset != newEntry.targetOffset) {
                stats.updateFixTarget++;
            }
            toUpdate[foundWay] = newEntry;
            toUpdate[foundWay].rrpv = srripTouch();
            stats.updateExisting++;

            DPRINTF(BTBPDede,
                "BTBPDede: updated existing monitor entry at bank %d, index %d, way %d\n",
                alignedBankIdx, monitorIdx, foundWay);

            if (foundVictimWay != numVictimCacheSets) {
                DPRINTF(BTBPDede,
                    "BTBPDede: deduplicating victim cache entry at bank %d, way %d\n",
                    alignedBankIdx, foundVictimWay);
                victimCache[alignedBankIdx][foundVictimWay].valid = false;
            }
        } else if (foundVictimWay != numVictimCacheSets) {
            auto &victimEntry = victimCache[alignedBankIdx][foundVictimWay];
            if (victimEntry.targetOffset != newEntry.targetOffset) {
                stats.updateFixTarget++;
            }
            victimEntry = newEntry;
            victimEntry.tag = getVictimCacheTag(monitorTag, monitorIdx);
            victimEntry.rrpv = srripTouch();
            stats.updateInVC++;

            DPRINTF(BTBPDede,
                "BTBPDede: updated existing victim cache entry at bank %d, way %d\n",
                alignedBankIdx, foundVictimWay);
        } else {
            if (!stream.exeTaken &&
                execAttr.branchType == BranchAttribute::BranchTypeEnum::Conditional) {
                DPRINTF(BTBPDede,
                    "BTBPDede: skip allocation for not taken conditional branch\n");
                continue;
            }

            stats.updateMissTimes++;

            unsigned evictWay = numWays;
            for (unsigned i = 0; i < numWays; ++i) {
                auto &entry = toUpdate[i];
                if (!entry.valid) {
                    evictWay = i;
                    stats.updateFoundEmptyTimes++;
                    DPRINTF(BTBPDede,
                        "BTBPDede: found empty way %d for bank %d idx %#lx\n",
                        evictWay, alignedBankIdx, monitorIdx);
                    break;
                }
            }

            if (evictWay == numWays) {
                stats.updateEvictTimes++;
                evictWay = findOrSelectSRRIPWay(toUpdate);
                stats.updateReplace++;

                DPRINTF(BTBPDede, "BTBPDede: evicting way %d for bank %d idx %#lx\n",
                    evictWay, alignedBankIdx, monitorIdx);
            }

            const auto &toEvictEntry = toUpdate[evictWay];
            if (toEvictEntry.valid && evictWay < numWays) {
                stats.updateReplaceValidOne++;
            }

            if (toEvictEntry.valid && victimCacheEntries != 0) {
                unsigned victimWay = numVictimCacheSets;
                for (unsigned way = 0; way < numVictimCacheSets; way++) {
                    auto &victimEntry = victimCache[alignedBankIdx][way];
                    if (!victimEntry.valid) {
                        victimWay = way;
                        DPRINTF(BTBPDede,
                            "BTBPDede: found empty victim cache way %d for bank %d\n",
                            victimWay, alignedBankIdx);
                        break;
                    }
                    if (victimEntry.tag !=
                        getVictimCacheTag(toEvictEntry.tag, monitorIdx)) continue;
                    if (victimEntry.position != toEvictEntry.position) continue;
                    victimWay = way;
                    break;
                }

                if (victimWay == numVictimCacheSets) {
                    victimWay = findOrSelectSRRIPWay(victimCache[alignedBankIdx]);
                    DPRINTF(BTBPDede,
                        "BTBPDede: evicting victim cache way %d for bank %d\n",
                        victimWay, alignedBankIdx);
                }

                auto &victimEntry = victimCache[alignedBankIdx][victimWay];
                victimEntry = toEvictEntry;
                victimEntry.tag = getVictimCacheTag(toEvictEntry.tag, monitorIdx);
                victimEntry.rrpv = insertRRPV;
            }

            toUpdate[evictWay] = newEntry;
            toUpdate[evictWay].rrpv = insertRRPV;
        }
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
        "pagePointerWay:%u, ctr:%d, carry:%d, rrpv:%u, attr:(branchType:%d, rasAction:%d)\n",
        e.getOffsetBits(), e.isUsePagePointer(), e.valid, e.isCrossPage,
        e.position, e.tag, e.targetOffset, e.pageTableSet,
        e.pageTableWay, e.ctr, (int)e.targetCarry.targetCarry, e.rrpv,
        (int)e.attr.branchType, (int)e.attr.rasAction);
}

void BTBPDede::printPageEntry(const PageEntry& e) {
    DPRINTF(BTBPDede, "PageEntry: valid:%d, tag:%#lx, regionWay:%#lx, rrpv:%u\n",
        e.valid, e.tag, e.regionWay, e.rrpv);
}

void BTBPDede::getAndSetNewBTBEntry(FetchTarget &stream)
{
    DPRINTF(BTBPDede, "getAndSetNewBTBEntry called for pc %#lx\n", stream.startPC);
    auto meta = std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);
    auto &predBTBEntries = meta->btbEntries;

    bool pred_branch_hit = false;
    BTBEntry entry_to_write = BTBEntry();
    for (auto &e: predBTBEntries) {
        if (stream.exeBranchInfo == e) {
            pred_branch_hit = true;
            entry_to_write = e;
            break;
        }
    }
    bool is_old_entry = pred_branch_hit;

    if (!pred_branch_hit && stream.exeTaken) {
        DPRINTF(BTBPDede, "Creating new BTB entry for pc %#lx\n", stream.exeBranchInfo.pc);
        BTBEntry new_entry = BTBEntry(stream.exeBranchInfo);
        new_entry.valid = true;
        if (new_entry.isCond) {
            new_entry.alwaysTaken = true;
            new_entry.ctr = 0;
            stats.newEntryWithCond++;
        } else {
            stats.newEntryWithUncond++;
        }
        stats.newEntry++;
        entry_to_write = new_entry;
        entry_to_write.resolved = stream.exeBranchInfo.resolved;
        is_old_entry = false;
    } else {
        DPRINTF(BTBPDede, "Not creating new entry: pred_branch_hit=%d, stream.exeTaken=%d\n",
                pred_branch_hit, stream.exeTaken);
    }

    stream.updateNewBTBEntry = entry_to_write;
    stream.updateIsOldEntry = is_old_entry;
}

void BTBPDede::commitBranch(const FetchTarget &stream, const DynInstPtr &inst) {
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

    bool hitBranchTaken = stream.exeTaken && stream.getControlPC() == pc;

    // unsigned targetDiffBits = getTargetDiffBits(pc, npc);

    stats.totalBranchHits += branchHit;
    stats.allBranchHits += branchHit;
    stats.totalBranchMisses += !branchHit;
    stats.allBranchMisses += !branchHit;

    if (branchHit) {
        if (hitBranchTaken) {
            stats.allBranchHitTakens++;
        } else {
            stats.allBranchHitNotTakens++;
        }
        if (inst->isCondCtrl()) {
            stats.condHits++;
            if (hitBranchTaken) {
                stats.condHitTakens++;
            } else {
                stats.condHitNotTakens++;
            }

            bool pred_taken = entry.ctr >= 0;
            if (pred_taken == hitBranchTaken) {
                stats.condPredCorrect++;
            } else {
                stats.condPredWrong++;
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
                bool found_raw_entry = false;
                bool entry_cross_page = false;
                bool entry_use_page_pointer = false;
                TargetCarry::TargetCarryEnum entry_carry = TargetCarry::TargetCarryEnum::None;
                unsigned offset = pc & (blockSize - 1);
                bool is_rvc = (entry.size == 2);
                unsigned aligned_position = is_rvc ? (offset >> 1) : ((offset + 2) >> 1);
                Addr monitor_tag = getMonitorTag(pc);
                for (const auto &bank : rawEntries) {
                    for (const auto &me : bank) {
                        if (!me.valid) {
                            continue;
                        }
                        if (me.position == aligned_position && me.tag == monitor_tag) {
                            found_raw_entry = true;
                            entry_cross_page = me.isCrossPage;
                            entry_use_page_pointer = me.isUsePagePointer();
                            entry_carry = me.targetCarry.targetCarry;
                            break;
                        }
                    }
                    if (found_raw_entry) {
                        break;
                    }
                }
                if (found_raw_entry) {
                    stats.indirectMetaFound++;
                } else {
                    stats.indirectMetaNotFound++;
                }
                if (found_raw_entry && entry_cross_page) {
                    stats.indirectHitCrossPage++;
                } else if (found_raw_entry) {
                    stats.indirectHitNonCrossPage++;
                }
                Addr pred_target = entry.target;
                if (pred_target == npc) {
                    stats.indirectPredCorrect++;
                } else {
                    stats.indirectPredWrong++;
                    if (found_raw_entry && entry_cross_page) {
                        stats.indirectPredWrongCrossPage++;
                    } else if (found_raw_entry) {
                        stats.indirectPredWrongNonCrossPage++;
                    }
                    if (found_raw_entry && !entry_cross_page) {
                        switch (entry_carry) {
                          case TargetCarry::TargetCarryEnum::Fit:
                            stats.indirectPredWrongCarryFit++;
                            break;
                          case TargetCarry::TargetCarryEnum::PlusOne:
                            stats.indirectPredWrongCarryPlusOne++;
                            break;
                          case TargetCarry::TargetCarryEnum::MinusOne:
                            stats.indirectPredWrongCarryMinusOne++;
                            break;
                          case TargetCarry::TargetCarryEnum::None:
                          default:
                            stats.indirectPredWrongCarryNone++;
                            break;
                        }
                    }
                    if (found_raw_entry) {
                        if (entry_use_page_pointer) {
                            stats.indirectPredWrongUsePagePointer++;
                        } else {
                            stats.indirectPredWrongNoPagePointer++;
                        }
                    }
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
        if (hitBranchTaken) {
            stats.allBranchMissTakens++;
        } else {
            stats.allBranchMissNotTakens++;
        }
        if (inst->isCondCtrl()) {
            stats.condMisses++;
            if (hitBranchTaken) {
                stats.condMissTakens++;
                stats.condPredWrong++;

            } else {
                stats.condMissNotTakens++;
                stats.condPredCorrect++;
            }
        }
        if (inst->isUncondCtrl()) {
            stats.uncondMisses++;
        }
        // ignore non-speculative branches (e.g. syscall)
        if (!inst->isNonSpeculative()) {
            if (inst->isIndirectCtrl()) {
                stats.indirectMisses++;
                stats.indirectMetaNotFound++;
                stats.indirectPredWrong++;
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
    ADD_STAT(newEntry, statistics::units::Count::get(), "number of new btb entries generated"),
    ADD_STAT(newEntryWithCond, statistics::units::Count::get(),
        "number of new btb entries generated with conditional branch"),
    ADD_STAT(newEntryWithUncond, statistics::units::Count::get(),
        "number of new btb entries generated with unconditional branch"),
    ADD_STAT(predTimes, statistics::units::Count::get(), "Number of predictions made"),
    ADD_STAT(predMissTimes, statistics::units::Count::get(), "Number of prediction misses"),
    ADD_STAT(predHitTimes, statistics::units::Count::get(), "Number of prediction hits"),
    ADD_STAT(predHitEntries, statistics::units::Count::get(), "Number of predicted entries hit"),
    ADD_STAT(predHitVictimTimes, statistics::units::Count::get(),
        "Number of predicted times from victim cache"),
    ADD_STAT(predHitVictimEntries, statistics::units::Count::get(),
        "Number of predicted entries from victim cache"),
    ADD_STAT(updateTimes, statistics::units::Count::get(), "Number of updates made"),
    ADD_STAT(updateMiss, statistics::units::Count::get(), "misses encountered on update"),
    ADD_STAT(updateHit, statistics::units::Count::get(), "hits encountered on update"),
    ADD_STAT(updateMissTimes, statistics::units::Count::get(), "Number of update misses"),
    ADD_STAT(updateFoundEmptyTimes, statistics::units::Count::get(),"Number of update where an empty entry was found"),
    ADD_STAT(updateEvictTimes, statistics::units::Count::get(),"Number of update where an entry was evicted"),
    ADD_STAT(updateHitTimes, statistics::units::Count::get(), "Number of update hits"),
    ADD_STAT(updateHitVictimTimes, statistics::units::Count::get(),
        "Number of update hits from victim cache"),
    ADD_STAT(updateMultiHitTimes, statistics::units::Count::get(), "Number of update multi-hits"),
    ADD_STAT(updateExisting, statistics::units::Count::get(), "existing entries updated"),
    ADD_STAT(updateReplace, statistics::units::Count::get(), "entries replaced"),
    ADD_STAT(updateReplaceValidOne, statistics::units::Count::get(),
        "entries replaced with valid entry"),
    ADD_STAT(updateInVC, statistics::units::Count::get(), "entries updated in victim cache"),
    ADD_STAT(updateTotal, statistics::units::Count::get(), "total number of entries updated"),
    ADD_STAT(updateFixTarget, statistics::units::Count::get(),
        "the number of fix entries target when update"),
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
    ADD_STAT(allBranchHits, statistics::units::Count::get(),
        "all types of branches committed that was predicted hit"),
    ADD_STAT(totalBranchHits, statistics::units::Count::get(), "Total number of branch hits in BTB"),
    ADD_STAT(allBranchHitTakens, statistics::units::Count::get(),
        "all types of taken branches committed was that predicted hit"),
    ADD_STAT(allBranchHitNotTakens, statistics::units::Count::get(),
        "all types of not taken branches committed was that predicted hit"),
    ADD_STAT(allBranchMisses, statistics::units::Count::get(),
        "all types of branches committed that was predicted miss"),
    ADD_STAT(totalBranchMisses, statistics::units::Count::get(), "Total number of branch misses in BTB"),
    ADD_STAT(allBranchMissTakens, statistics::units::Count::get(),
        "all types of taken branches committed was that predicted miss"),
    ADD_STAT(allBranchMissNotTakens, statistics::units::Count::get(),
        "all types of not taken branches committed was that predicted miss"),
    ADD_STAT(condHits, statistics::units::Count::get(), "Number of conditional branch hits"),
    ADD_STAT(condHitTakens, statistics::units::Count::get(),
        "taken conditional branches committed was that predicted hit"),
    ADD_STAT(condHitNotTakens, statistics::units::Count::get(),
        "not taken conditional branches committed was that predicted hit"),
    ADD_STAT(condMisses, statistics::units::Count::get(), "Number of conditional branch misses"),
    ADD_STAT(condMissTakens, statistics::units::Count::get(),
        "taken conditional branches committed was that predicted miss"),
    ADD_STAT(condMissNotTakens, statistics::units::Count::get(),
        "not taken conditional branches committed was that predicted miss"),
    ADD_STAT(condPredCorrect, statistics::units::Count::get(),
        "conditional branches committed was that correctly predicted by btb"),
    ADD_STAT(condPredWrong, statistics::units::Count::get(),
        "conditional branches committed was that mispredicted by btb"),
    ADD_STAT(uncondHits, statistics::units::Count::get(), "Number of unconditional branch hits"),
    ADD_STAT(uncondMisses, statistics::units::Count::get(), "Number of unconditional branch misses"),
    ADD_STAT(indirectHits, statistics::units::Count::get(), "Number of indirect branch hits"),
    ADD_STAT(indirectMisses, statistics::units::Count::get(), "Number of indirect branch misses"),
    ADD_STAT(indirectPredCorrect, statistics::units::Count::get(),
        "indirect branches committed whose target was correctly predicted by btb"),
    ADD_STAT(indirectPredWrong, statistics::units::Count::get(),
        "indirect branches committed whose target was mispredicted by btb"),
    ADD_STAT(indirectMetaFound, statistics::units::Count::get(),
        "indirect branches whose monitor meta entry was found at commit"),
    ADD_STAT(indirectMetaNotFound, statistics::units::Count::get(),
        "indirect branches whose monitor meta entry was not found at commit"),
    ADD_STAT(indirectHitCrossPage, statistics::units::Count::get(),
        "indirect hit branches predicted by cross-page entries"),
    ADD_STAT(indirectHitNonCrossPage, statistics::units::Count::get(),
        "indirect hit branches predicted by non-cross-page entries"),
    ADD_STAT(indirectPredWrongCrossPage, statistics::units::Count::get(),
        "indirect target mispredictions from cross-page entries"),
    ADD_STAT(indirectPredWrongNonCrossPage, statistics::units::Count::get(),
        "indirect target mispredictions from non-cross-page entries"),
    ADD_STAT(indirectPredWrongCarryFit, statistics::units::Count::get(),
        "indirect target mispredictions with carry Fit"),
    ADD_STAT(indirectPredWrongCarryPlusOne, statistics::units::Count::get(),
        "indirect target mispredictions with carry PlusOne"),
    ADD_STAT(indirectPredWrongCarryMinusOne, statistics::units::Count::get(),
        "indirect target mispredictions with carry MinusOne"),
    ADD_STAT(indirectPredWrongCarryNone, statistics::units::Count::get(),
        "indirect target mispredictions with carry None"),
    ADD_STAT(indirectPredWrongUsePagePointer, statistics::units::Count::get(),
        "indirect target mispredictions on entries using page pointer"),
    ADD_STAT(indirectPredWrongNoPagePointer, statistics::units::Count::get(),
        "indirect target mispredictions on entries not using page pointer"),
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
