#include <algorithm>

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
    numAlignBanks = predictWidth / blockSize; // 2 align banks

    // Initialize monitor btb
    numSets = numEntries / (numWays * numAlignBanks);
    monitorBTB.resize(numAlignBanks);
    monitorRrpv.resize(numAlignBanks);
    for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
        monitorBTB[bank].resize(numSets);
        monitorRrpv[bank].resize(numSets);
        for (unsigned set = 0; set < numSets; ++set) {
            monitorBTB[bank][set].resize(numWays);
            monitorRrpv[bank][set].resize(numWays * shortSlots);
            std::for_each(monitorRrpv[bank][set].begin(), monitorRrpv[bank][set].end(),
                [](unsigned &rrpv) { rrpv = monitorMaxRrpv; });
        }
    }
    // Initialize page btb
    numPageSets = numPageEntries / numPageWays;
    pageBTB.resize(numPageSets);
    pageRrpv.resize(numPageSets);
    for (unsigned set = 0; set < numPageSets; ++set) {
        pageBTB[set].resize(numPageWays);
        pageRrpv[set].resize(numPageWays);
        std::for_each(pageRrpv[set].begin(), pageRrpv[set].end(),
            [](unsigned &rrpv) { rrpv = pageMaxRrpv; });
    }

    // Initialize region btb
    numRegionSets = numRegionEntries / numRegionWays;
    assert(numRegionSets == 1);
    regionBTB.resize(numRegionSets);
    regionRrpv.resize(numRegionSets);
    for (unsigned set = 0; set < numRegionSets; ++set) {
        regionBTB[set].resize(numRegionWays);
        regionRrpv[set].resize(numRegionWays);
        std::for_each(regionRrpv[set].begin(), regionRrpv[set].end(),
            [](unsigned &rrpv) { rrpv = regionMaxRrpv; });
    }

    // Initialize victim cache
    // numVictimCacheSets = victimCacheEntries / numAlignBanks;
    // victimCache.resize(numAlignBanks);
    // for (unsigned bank = 0; bank < numAlignBanks; ++bank) {
    //     victimCache[bank].resize(numVictimCacheSets);
    // }


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

unsigned BTBPDede::getPhysicalAlignBankIdx(Addr pc, unsigned logicBankIdx)
{
    // Rotate align bank index based on PC bits to reduce conflicts
    unsigned alignBankWidth = floorLog2(blockSize);
    unsigned rotation = (pc >> alignBankWidth) & (numAlignBanks - 1);
    return (logicBankIdx + rotation) % numAlignBanks;
}

Addr BTBPDede::getShortSlotTarget(Addr pc, const MonitorShortSlot &slot, unsigned targetBits)
{
    Addr validTargetBits = targetBits + instShiftAmt;
    Addr validTargetMask = mask(validTargetBits);
    Addr targetLower = slot.bi.target & validTargetMask;
    Addr pcUpper = pc & ~validTargetMask;
    return pcUpper | targetLower;
}

Addr BTBPDede::getLongSlotTarget(Addr pc, const MonitorLongSlot &slot, unsigned targetBits)
{
    Addr validTargetBits = targetBits + instShiftAmt;
    Addr validTargetMask = mask(validTargetBits);
    Addr targetLower = slot.bi.target & validTargetMask;
    Addr pcUpper = pc & ~validTargetMask;
    Addr pcUpperOverflow = pcUpper + (1ULL << validTargetBits);
    Addr pcUpperUnderflow = pcUpper - (1ULL << validTargetBits);
    if (!slot.isCrossPage) {
        if (slot.isOverflow) {
            return pcUpperOverflow | targetLower;
        } else if (slot.isUnderflow) {
            return pcUpperUnderflow | targetLower;
        } else {
            return pcUpper | targetLower;
        }
    } else {
        // access Page/Region-BTB to get the full target
        if (slot.index >= numPageSets || slot.way >= numPageWays) {
            DPRINTF(BTBPDede,
                "BTBPDede: invalid page pointer idx=%#lx way=%#lx, fallback to non-cross-page decode\n",
                slot.index, slot.way);
            return pcUpper | targetLower;
        }

        const auto &pageBTBEntry = pageBTB[slot.index][slot.way];
        if (!pageBTBEntry.valid || pageBTBEntry.way >= numRegionWays ||
            !regionBTB[0][pageBTBEntry.way].valid) {
            DPRINTF(BTBPDede,
                "BTBPDede: stale page/region pointer idx=%#lx way=%#lx rway=%#lx, fallback to non-cross-page decode\n",
                slot.index, slot.way, pageBTBEntry.way);
            return pcUpper | targetLower;
        }

        Addr vpnLower = pageBTBEntry.vpnLower << (validTargetBits);
        Addr vpnUpper = regionBTB[0][pageBTBEntry.way].vpnUpper << (pageBits + validTargetBits);
        return vpnUpper | vpnLower | targetLower;
    }
}

Addr BTBPDede::getMonitorBTBIdx(Addr pc)
{
    unsigned fetchBlockWidth = floorLog2(predictWidth);
    Addr idx = (pc >> fetchBlockWidth) & (numSets - 1);
    return idx;
}

Addr BTBPDede::getMonitorBTBTag(Addr pc)
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
        unsigned phyBankIdx = getPhysicalAlignBankIdx(pc, i);
        Addr alignedAddr = alignedStartAddr + blockSize * i;
        Addr idx = getMonitorBTBIdx(alignedAddr);
        MonitorSet monitorSet = monitorBTB[phyBankIdx][idx];
        res[phyBankIdx] = monitorSet;
    }

    return res;
}

Addr BTBPDede::getPageBTBIdx(Addr target, unsigned targetBits)
{
    unsigned setWidth = floorLog2(numPageSets);
    unsigned validTargetBits = targetBits + instShiftAmt;
    Addr idxFull = target >> validTargetBits;

    // use idxFull[setWidth * 2 - 1:setWidth] ^ idxFull[setWidth - 1:0] to reduce conflicts
    Addr idxHigher = (idxFull >> setWidth) & mask(setWidth);
    Addr idxLower = idxFull & mask(setWidth);
    Addr idx = idxHigher ^ idxLower;

    return idx;
}

Addr BTBPDede::getVpnLower(Addr target, unsigned targetBits) {
    unsigned validTargetBits = targetBits + instShiftAmt;
    Addr fullTag = target >> validTargetBits;
    return fullTag & mask(pageBits);
}

Addr BTBPDede::getVpnUpper(Addr target, unsigned targetBits) {
    unsigned validTargetBits = targetBits + instShiftAmt;
    return target >> (pageBits + validTargetBits);
}

// Addr BTBPDede::getVictimCacheTag(Addr monitorTag, Addr monitorIdx)
// {
//     unsigned monitorSetWidth = floorLog2(numSets);
//     Addr victimTag = (monitorTag << monitorSetWidth) | monitorIdx;
//     return victimTag;
// }

std::vector<BTBEntry> BTBPDede::processMonitorEntries(Addr pc, const std::vector<MonitorSet> &originEntries)
{
    auto monitorEntries = originEntries;
    std::vector<BTBEntry> btbEntries;
    Addr endPc = (pc + predictWidth) & ~mask(floorLog2(predictWidth) - 1);

    DPRINTF(BTBPDede,
        "BTBPDede: processMonitorEntries startPC=%#lx endPC=%#lx alignedStart=%#lx\n",
        pc, endPc, pc & ~(blockSize - 1));

    // collect all valid entries
    for (unsigned i = 0; i < numAlignBanks; ++i) {
        unsigned phyBankIdx = getPhysicalAlignBankIdx(pc, i);
        auto &bank = monitorEntries[phyBankIdx];
        Addr alignedAddr = (pc & ~(blockSize - 1)) + blockSize * i;
        Addr monitorBTBIdx = getMonitorBTBIdx(alignedAddr);
        Addr monitorBTBTag = getMonitorBTBTag(alignedAddr);

        DPRINTF(BTBPDede,
            "BTBPDede: inspect logicBank=%u phyBank=%u alignedAddr=%#lx idx=%#lx tag=%#lx\n",
            i, phyBankIdx, alignedAddr, monitorBTBIdx, monitorBTBTag);

        for (unsigned way = 0; way < numWays; ++way) {
            MonitorEntry &entry = bank[way];

            auto checkValidEntry = [&](bool valid, Addr tag, Addr branchPc) -> bool {
                if (!valid) {
                    DPRINTF(BTBPDede,
                        "BTBPDede: reject way=%u alignedAddr=%#lx branchPc=%#lx \
                        reason=invalid\n",
                        way, alignedAddr, branchPc);
                    return false;
                }
                if (branchPc < pc || branchPc >= endPc) {
                    DPRINTF(BTBPDede,
                        "BTBPDede: reject way=%u alignedAddr=%#lx branchPc=%#lx \
                        reason=out_of_range start=%#lx end=%#lx\n",
                        way, alignedAddr, branchPc, pc, endPc);
                    return false;
                }
                if (tag != monitorBTBTag) {
                    DPRINTF(BTBPDede,
                        "BTBPDede: reject way=%u alignedAddr=%#lx branchPc=%#lx \
                        reason=tag_mismatch entryTag=%#lx expectTag=%#lx\n",
                        way, alignedAddr, branchPc, tag, monitorBTBTag);
                    return false;
                }
                return true;
            };

            bool hitLongSlot = false;
            bool hitShortSlot[shortSlots] = {false};

            if (!entry.fused) {
                for (unsigned slot = 0; slot < shortSlots; ++slot) {
                    auto shortSlot = entry.shortSlots[slot];
                    if (!checkValidEntry(shortSlot.valid, entry.tag, shortSlot.bi.pc)) continue;
                    hitShortSlot[slot] = true;
                    Addr alignedBranchPc = shortSlot.bi.pc & ~(blockSize - 1);
                    assert(alignedBranchPc == alignedAddr);

                    BTBEntry btbEntry;
                    btbEntry.valid = true;
                    btbEntry.pc = shortSlot.bi.pc;
                    btbEntry.tag = entry.tag;
                    btbEntry.target = getShortSlotTarget(shortSlot.bi.pc, shortSlot, shortSlotTargetBits);
                    btbEntry.isCond = shortSlot.bi.isCond;
                    btbEntry.isDirect = shortSlot.bi.isDirect;
                    btbEntry.isIndirect = shortSlot.bi.isIndirect;
                    btbEntry.isCall = shortSlot.bi.isCall;
                    btbEntry.isReturn = shortSlot.bi.isReturn;
                    btbEntry.size = shortSlot.bi.size;
                    btbEntry.alwaysTaken = shortSlot.alwaysTaken;
                    btbEntry.ctr = shortSlot.ctr;
                    btbEntries.push_back(btbEntry);
                    DPRINTF(BTBPDede, "BTBPDede: use short slot %d way %d for bank %d alignedAddr %#lx\n",
                        slot,
                        way,
                        phyBankIdx,
                        alignedAddr
                    );
                }
            } else {
                auto longSlot = entry.longSlot;
                // long slot use short slot0's valid bit to indicate valid
                if (!checkValidEntry(entry.shortSlots[0].valid, entry.tag, longSlot.bi.pc)) continue;
                hitLongSlot = true;
                Addr alignedBranchPc = longSlot.bi.pc & ~(blockSize - 1);
                assert(alignedBranchPc == alignedAddr);

                BTBEntry btbEntry;
                btbEntry.valid = true;
                btbEntry.pc = longSlot.bi.pc;
                btbEntry.tag = entry.tag;
                btbEntry.target = getLongSlotTarget(longSlot.bi.pc, longSlot, longSlotTargetBits);
                btbEntry.isCond = longSlot.bi.isCond;
                btbEntry.isDirect = longSlot.bi.isDirect;
                btbEntry.isIndirect = longSlot.bi.isIndirect;
                btbEntry.isCall = longSlot.bi.isCall;
                btbEntry.isReturn = longSlot.bi.isReturn;
                btbEntry.size = longSlot.bi.size;
                // for conditional branches, use short slot0's counter when fused.
                btbEntry.alwaysTaken = entry.shortSlots[0].alwaysTaken;
                btbEntry.ctr = entry.shortSlots[0].ctr;
                btbEntries.push_back(btbEntry);
                DPRINTF(BTBPDede, "BTBPDede: use long slot entry way %d for bank %d alignedAddr %#lx\n",
                    way,
                    phyBankIdx,
                    alignedAddr
                );
            }

            if (hitLongSlot) { // fused
                monitorRrpv[phyBankIdx][monitorBTBIdx][way * shortSlots] = 0;
                monitorRrpv[phyBankIdx][monitorBTBIdx][way * shortSlots + 1] = 0;
                if (entry.longSlot.isCrossPage) { // cross page
                    Addr pageBTBIdx = entry.longSlot.index;
                    Addr pageBTBWay = entry.longSlot.way;
                    if (pageBTBIdx < numPageSets && pageBTBWay < numPageWays) {
                        auto &pageBTBEntry = pageBTB[pageBTBIdx][pageBTBWay];
                        if (pageBTBEntry.valid && pageBTBEntry.way < numRegionWays &&
                            regionBTB[0][pageBTBEntry.way].valid) {
                            pageRrpv[pageBTBIdx][pageBTBWay] = 0;
                            regionRrpv[0][pageBTBEntry.way] = 0;
                        }
                    }
                }
            } else {
                for (unsigned slot = 0; slot < shortSlots; ++slot) {
                    if (hitShortSlot[slot]) {
                        monitorRrpv[phyBankIdx][monitorBTBIdx][way * shortSlots + slot] = 0;
                    }
                }
            }
        }
    }

    std::sort(btbEntries.begin(), btbEntries.end(),
        [](const BTBEntry &a, const BTBEntry &b) {
            return a.pc < b.pc;
        }
    );

    DPRINTF(BTBPDede, "BTBPDede: final %zu entries for startPC %#lx\n", btbEntries.size(), pc);
    for (const auto &e : btbEntries) {
        DPRINTF(BTBPDede,
            "BTBPDede: final entry pc=%#lx target=%#lx cond=%d indirect=%d call=%d return=%d alwaysTaken=%d ctr=%d\n",
            e.pc, e.target, e.isCond, e.isIndirect, e.isCall, e.isReturn,
            e.alwaysTaken, e.ctr);
    }

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

    DPRINTF(BTBPDede,
        "BTBPDede: prepareUpdateEntries startPC=%#lx controlPC=%#lx \
        exeTaken=%d updateIsOldEntry=%d existing=%zu\n",
        stream.startPC, stream.getControlPC(), stream.exeTaken,
        stream.updateIsOldEntry, all_entries.size());
    for (const auto &e : all_entries) {
        DPRINTF(BTBPDede,
            "BTBPDede: existing update entry pc=%#lx target=%#lx resolved=%d \
            cond=%d indirect=%d alwaysTaken=%d ctr=%d\n",
            e.pc, e.target, e.resolved, e.isCond, e.isIndirect,
            e.alwaysTaken, e.ctr);
    }

    if (!stream.updateIsOldEntry) {
        BTBEntry potential_new_entry = stream.updateNewBTBEntry;
        bool new_entry_taken =
            stream.exeTaken && stream.getControlPC() == potential_new_entry.pc;
        if (!new_entry_taken) {
            potential_new_entry.alwaysTaken = false;
        }
        all_entries.push_back(potential_new_entry);
        DPRINTF(BTBPDede,
            "BTBPDede: appended new entry pc=%#lx target=%#lx taken=%d \
            cond=%d indirect=%d alwaysTaken=%d ctr=%d\n",
            potential_new_entry.pc, potential_new_entry.target, new_entry_taken,
            potential_new_entry.isCond, potential_new_entry.isIndirect,
            potential_new_entry.alwaysTaken, potential_new_entry.ctr);
    }

    if (getResolvedUpdate()) {
        auto remove_it = std::remove_if(
            all_entries.begin(),
            all_entries.end(),
            [](const BTBEntry &e) { return !e.resolved; });
        all_entries.erase(remove_it, all_entries.end());
    }

    DPRINTF(BTBPDede, "BTBPDede: final update entry count=%zu\n", all_entries.size());
    for (const auto &e : all_entries) {
        DPRINTF(BTBPDede,
            "BTBPDede: final update entry pc=%#lx target=%#lx resolved=%d \
            cond=%d indirect=%d alwaysTaken=%d ctr=%d\n",
            e.pc, e.target, e.resolved, e.isCond, e.isIndirect,
            e.alwaysTaken, e.ctr);
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
        DPRINTF(BTBPDede,
            "BTBPDede: update miss exePC=%#lx controlPC=%#lx exeTaken=%d\n",
            stream.exeBranchInfo.pc, stream.getControlPC(), stream.exeTaken);
        stats.updateMiss++;
    } else {
        DPRINTF(BTBPDede,
            "BTBPDede: update hit exePC=%#lx controlPC=%#lx exeTaken=%d\n",
            stream.exeBranchInfo.pc, stream.getControlPC(), stream.exeTaken);
        stats.updateHit++;
    }
}

unsigned distance(Addr pc, Addr target) {
    Addr diff = pc ^ target;
    unsigned dist = 0;
    while (diff) {
        diff = diff >> 1;
        dist++;
    }
    return dist;
}

bool BTBPDede::isOverflow(Addr pc, Addr target, unsigned targetBits) {
    Addr validTargetBits = targetBits + instShiftAmt;
    Addr validTargetMask = mask(validTargetBits);
    Addr targetUpper = target & ~validTargetMask;
    Addr pcUpper = pc & ~validTargetMask;
    Addr pcUpperOverflow = pcUpper + (1ULL << validTargetBits);
    return targetUpper == pcUpperOverflow;
}

bool BTBPDede::isUnderflow(Addr pc, Addr target, unsigned targetBits) {
    Addr validTargetBits = targetBits + instShiftAmt;
    Addr validTargetMask = mask(validTargetBits);
    Addr targetUpper = target & ~validTargetMask;
    Addr pcUpper = pc & ~validTargetMask;
    Addr pcUpperUnderflow = pcUpper - (1ULL << validTargetBits);
    return targetUpper == pcUpperUnderflow;
}

bool BTBPDede::isCrossPage(Addr pc, Addr target) {
    Addr vpnBits = ceilLog2(pageSize);
    Addr pcVpn = pc >> vpnBits;
    Addr targetVpn = target >> vpnBits;
    return pcVpn != targetVpn;
}

void BTBPDede::updateResolvedEntry(const BTBEntry &entry, const FetchTarget &stream) {
    Addr pc = entry.pc;
    Addr target = entry.target;
    bool isMispredict = stream.squashType == SQUASH_CTRL && stream.squashPC == pc;
    bool thisBranchTaken = stream.exeTaken && stream.exeBranchInfo.pc == pc;

    unsigned dist = distance(pc, target);

    bool canUseShortSlot = dist <= (shortSlotTargetBits + instShiftAmt);

    bool updateIsFused = !canUseShortSlot;
    if (entry.isIndirect) updateIsFused = true;

    unsigned bankIdx = getPhysicalAlignBankIdx(pc, 0);
    unsigned monitorBTBIdx = getMonitorBTBIdx(pc);
    unsigned monitorBTBTag = getMonitorBTBTag(pc);
    unsigned pageBTBIdx = getPageBTBIdx(target, longSlotTargetBits);
    Addr vpnLower = getVpnLower(target, longSlotTargetBits);
    Addr vpnUpper = getVpnUpper(target, longSlotTargetBits);

    DPRINTF(BTBPDede,
        "BTBPDede: updateResolvedEntry pc=%#lx target=%#lx thisTaken=%d \
        mispredict=%d updateIsFused=%d bank=%u idx=%u tag=%#lx pageIdx=%u vpnLower=%#lx vpnUpper=%#lx\n",
        pc, target, thisBranchTaken, isMispredict, updateIsFused, bankIdx,
        monitorBTBIdx, monitorBTBTag, pageBTBIdx, vpnLower, vpnUpper);

    auto &toUpdateSet = monitorBTB[bankIdx][monitorBTBIdx];
    auto &toUpdateRrpvSet = monitorRrpv[bankIdx][monitorBTBIdx];

    unsigned foundWay = numWays;
    unsigned foundSlot = shortSlots;
    for (unsigned way = 0; way < numWays; ++way) {
        auto &entry = toUpdateSet[way];
        if (entry.tag != monitorBTBTag) continue;
        if (!entry.shortSlots[0].valid && !entry.shortSlots[1].valid) continue;
        if (!entry.fused) {
            for (unsigned slot = 0; slot < shortSlots; ++slot) {
                auto shortSlot = entry.shortSlots[slot];
                if (shortSlot.valid && shortSlot.bi.pc == pc) {
                    foundWay = way;
                    foundSlot = slot;
                    break;
                }
            }
        } else {
            auto longSlot = entry.longSlot;
            if (entry.shortSlots[0].valid && longSlot.bi.pc == pc) {
                foundWay = way;
                break;
            }
        }
    }

    unsigned foundPageWay = numPageWays;
    for (unsigned way = 0; way < numPageWays; ++way) {
        auto &pageBTBEntry = pageBTB[pageBTBIdx][way];
        if (!pageBTBEntry.valid) continue;
        if (pageBTBEntry.way >= numRegionWays) continue;
        auto &regionBTBEntry = regionBTB[0][pageBTBEntry.way];
        if (!regionBTBEntry.valid) continue;
        if (pageBTBEntry.vpnLower != vpnLower) continue;
        if (regionBTBEntry.vpnUpper != vpnUpper) continue;
        foundPageWay = way;
        break;
    }

    unsigned foundRegionWay = numRegionWays;
    for (unsigned way = 0; way < numRegionWays; ++way) {
        auto &entry = regionBTB[0][way];
        if (!entry.valid) continue;
        if (entry.vpnUpper != vpnUpper) continue;
        foundRegionWay = way;
        break;
    }

    if (foundWay == numWays) {
        stats.updateLookupMiss++;
    } else if (foundSlot != shortSlots) {
        stats.updateLookupHitShortSlot++;
    } else {
        stats.updateLookupHitLongSlot++;
    }

    DPRINTF(BTBPDede,
        "BTBPDede: update lookup result pc=%#lx foundWay=%u foundSlot=%u foundPageWay=%u foundRegionWay=%u\n",
        pc, foundWay, foundSlot, foundPageWay, foundRegionWay);

    auto writeFusedEntry = [&](MonitorEntry &toWrite) {
        toWrite.fused = true;
        toWrite.tag = monitorBTBTag;
        toWrite.shortSlots[0].valid = true;
        toWrite.longSlot.bi = BranchInfo(entry);
        toWrite.longSlot.bi.resolved = false;

        bool crossPage = isCrossPage(pc, target);
        bool overflow = isOverflow(pc, target, longSlotTargetBits);
        bool underflow = isUnderflow(pc, target, longSlotTargetBits);

        toWrite.longSlot.isCrossPage = crossPage && (!overflow && !underflow);
        toWrite.longSlot.isOverflow = overflow;
        toWrite.longSlot.isUnderflow = underflow;

        if (toWrite.longSlot.isCrossPage) {
            toWrite.longSlot.isOverflow = false;
            toWrite.longSlot.isUnderflow = false;

            auto &pageBTBSet = pageBTB[pageBTBIdx];
            auto &pageBTBSetRrpv = pageRrpv[pageBTBIdx];
            auto &regionBTBSet = regionBTB[0];
            auto &regionBTBSetRrpv = regionRrpv[0];
            unsigned victimPageWay = numPageWays;
            unsigned victimRegionWay = numRegionWays;

            unsigned writePageWay = foundPageWay;
            unsigned writeRegionWay = foundRegionWay;

            if (foundPageWay == numPageWays) { // page btb miss
                auto maxRrpvIt = std::max_element(pageBTBSetRrpv.begin(), pageBTBSetRrpv.end());
                unsigned maxRrpvWay = std::distance(pageBTBSetRrpv.begin(), maxRrpvIt);
                unsigned maxRrpv = *maxRrpvIt;
                victimPageWay = maxRrpvWay;
                writePageWay = victimPageWay;
                std::transform(pageBTBSetRrpv.begin(), pageBTBSetRrpv.end(), pageBTBSetRrpv.begin(),
                    [&](unsigned rrpv) { return rrpv + (pageMaxRrpv - maxRrpv); });
                pageBTBSetRrpv[victimPageWay] = pageMaxRrpv - 1;
            } else {
                pageBTBSetRrpv[writePageWay] = 0;
            }
            if (foundRegionWay == numRegionWays) { // region btb miss
                auto maxRrpvIt = std::max_element(regionBTBSetRrpv.begin(), regionBTBSetRrpv.end());
                unsigned maxRrpvWay = std::distance(regionBTBSetRrpv.begin(), maxRrpvIt);
                unsigned maxRrpv = *maxRrpvIt;
                victimRegionWay = maxRrpvWay;
                writeRegionWay = victimRegionWay;

                std::transform(regionBTBSetRrpv.begin(), regionBTBSetRrpv.end(), regionBTBSetRrpv.begin(),
                    [&](unsigned rrpv) { return rrpv + (regionMaxRrpv - maxRrpv); });
                regionBTBSetRrpv[victimRegionWay] = regionMaxRrpv - 1;
            } else {
                regionBTBSetRrpv[writeRegionWay] = 0;
            }
            toWrite.longSlot.index = pageBTBIdx;
            toWrite.longSlot.way = writePageWay;
            pageBTBSet[writePageWay].valid = true;
            pageBTBSet[writePageWay].vpnLower = vpnLower;
            pageBTBSet[writePageWay].way = writeRegionWay;
            regionBTBSet[writeRegionWay].valid = true;
            regionBTBSet[writeRegionWay].vpnUpper = vpnUpper;
        }
    };

    // entry update
    MonitorShortSlot *writtenSlot = nullptr;

    auto maxRrpvIt = std::max_element(toUpdateRrpvSet.begin(), toUpdateRrpvSet.end());
    unsigned maxRrpvDistance = std::distance(toUpdateRrpvSet.begin(), maxRrpvIt);
    unsigned maxRrpvDistanceBuddy = maxRrpvDistance ^ 1;
    unsigned maxRrpvWay = maxRrpvDistance / shortSlots;
    unsigned maxRrpv = *maxRrpvIt;

    if (foundWay == numWays) { // miss
        // check not taken conditional branch
        if (entry.isCond && !thisBranchTaken) {
            DPRINTF(BTBPDede,
                "BTBPDede: skip allocate not-taken conditional pc=%#lx\n", pc);
            return;
        }
        // check paritially invalid slot
        for (unsigned way = 0; way < numWays; ++way) {
            if (updateIsFused) continue; // fused entry cannot use partially invalid entry
            if (toUpdateSet[way].fused) continue;
            for (unsigned slot = 0; slot < shortSlots; ++slot) {
                if (toUpdateSet[way].shortSlots[slot].valid) continue;
                if (toUpdateSet[way].tag != monitorBTBTag) continue;

                stats.updateWritePartialInvalidSlot++;
                DPRINTF(BTBPDede,
                    "BTBPDede: allocate partial-invalid slot pc=%#lx way=%u slot=%u fused=%d\n",
                    pc, way, slot, updateIsFused);

                toUpdateSet[way].shortSlots[slot].valid = true;
                toUpdateSet[way].shortSlots[slot].bi = BranchInfo(entry);
                toUpdateSet[way].shortSlots[slot].bi.resolved = false;

                toUpdateRrpvSet[way * shortSlots + slot] = monitorMaxRrpv - 1;
                writtenSlot = &toUpdateSet[way].shortSlots[slot];
                goto _counter_update;
            }
        }
        // check invalid entry
        for (unsigned way = 0; way < numWays; ++way) {
            if (!toUpdateSet[way].shortSlots[0].valid && !toUpdateSet[way].shortSlots[1].valid) {
                stats.updateWriteInvalidWay++;
                DPRINTF(BTBPDede,
                    "BTBPDede: allocate invalid way pc=%#lx way=%u fused=%d\n",
                    pc, way, updateIsFused);
                writtenSlot = &toUpdateSet[way].shortSlots[0];
                if (updateIsFused) {
                    writeFusedEntry(toUpdateSet[way]);
                    toUpdateRrpvSet[way * shortSlots] = monitorMaxRrpv - 1;
                    toUpdateRrpvSet[way * shortSlots + 1] = monitorMaxRrpv - 1;
                } else {
                    toUpdateSet[way].fused = false;
                    toUpdateSet[way].tag = monitorBTBTag;
                    toUpdateSet[way].shortSlots[0].valid = true;
                    toUpdateSet[way].shortSlots[0].bi = BranchInfo(entry);
                    toUpdateSet[way].shortSlots[0].bi.resolved = false;
                    toUpdateRrpvSet[way * shortSlots] = monitorMaxRrpv - 1;
                }
                goto _counter_update;
            }
        }
        // if no invalid entry, need to replace the entry with max RRPV
        if (updateIsFused == toUpdateSet[maxRrpvWay].fused) {
            stats.updateWriteReplaceSameType++;
            DPRINTF(BTBPDede,
                "BTBPDede: replace same-type pc=%#lx victimWay=%u fused=%d maxRrpvDistance=%u buddy=%u\n",
                pc, maxRrpvWay, updateIsFused, maxRrpvDistance, maxRrpvDistanceBuddy);
            if (updateIsFused) { // write fused entry with fused entry
                writeFusedEntry(toUpdateSet[maxRrpvWay]);
                writtenSlot = &toUpdateSet[maxRrpvWay].shortSlots[0];
                std::transform(toUpdateRrpvSet.begin(), toUpdateRrpvSet.end(), toUpdateRrpvSet.begin(),
                    [&](unsigned rrpv) { return rrpv + (monitorMaxRrpv - maxRrpv); });
                toUpdateRrpvSet[maxRrpvDistance] = monitorMaxRrpv - 1;
                toUpdateRrpvSet[maxRrpvDistanceBuddy] = monitorMaxRrpv - 1;
            } else { // write unfused entry with unfused entry
                bool retagged = toUpdateSet[maxRrpvWay].tag != monitorBTBTag;
                toUpdateSet[maxRrpvWay].fused = false;
                toUpdateSet[maxRrpvWay].tag = monitorBTBTag;
                auto writeShortSlot = [&](MonitorShortSlot &slot) {
                    slot.valid = true;
                    slot.bi = BranchInfo(entry);
                    slot.bi.resolved = false;
                };
                auto invalidateShortSlot = [&](MonitorShortSlot &slot) {
                    slot = MonitorShortSlot();
                };
                if (maxRrpvDistance % shortSlots == 0) {
                    writeShortSlot(toUpdateSet[maxRrpvWay].shortSlots[0]);
                    if (retagged) {
                        invalidateShortSlot(toUpdateSet[maxRrpvWay].shortSlots[1]);
                    }
                    writtenSlot = &toUpdateSet[maxRrpvWay].shortSlots[0];
                } else {
                    writeShortSlot(toUpdateSet[maxRrpvWay].shortSlots[1]);
                    if (retagged) {
                        invalidateShortSlot(toUpdateSet[maxRrpvWay].shortSlots[0]);
                    }
                    writtenSlot = &toUpdateSet[maxRrpvWay].shortSlots[1];
                }

                std::transform(toUpdateRrpvSet.begin(), toUpdateRrpvSet.end(), toUpdateRrpvSet.begin(),
                    [&](unsigned rrpv) { return rrpv + (monitorMaxRrpv - maxRrpv); });
                toUpdateRrpvSet[maxRrpvDistance] = monitorMaxRrpv - 1;
            }
        } else if (updateIsFused && !toUpdateSet[maxRrpvWay].fused) { // write fused entry with unfused entry
            stats.updateWriteFuseOnUnfusedWay++;
            DPRINTF(BTBPDede,
                "BTBPDede: convert unfused->fused pc=%#lx victimWay=%u buddyRrpv=%u\n",
                pc, maxRrpvWay, toUpdateRrpvSet[maxRrpvDistanceBuddy]);
            unsigned buddyRrpv = toUpdateRrpvSet[maxRrpvDistanceBuddy];
            if (buddyRrpv > monitorMaxRrpv / 2) { // not recently used
                writeFusedEntry(toUpdateSet[maxRrpvWay]);
                writtenSlot = &toUpdateSet[maxRrpvWay].shortSlots[0];
                std::transform(toUpdateRrpvSet.begin(), toUpdateRrpvSet.end(), toUpdateRrpvSet.begin(),
                    [&](unsigned rrpv) { return rrpv + (monitorMaxRrpv - maxRrpv); });
                toUpdateRrpvSet[maxRrpvDistance] = monitorMaxRrpv - 1;
                toUpdateRrpvSet[maxRrpvDistanceBuddy] = monitorMaxRrpv - 1;
            } else { // recently used
                // check all fused entries to find the most rrpv one to replace
                unsigned fusedVictimWay = numWays;
                unsigned fusedVictimRrpv = 0;
                for (unsigned way = 0; way < numWays; ++way) {
                    if (!toUpdateSet[way].fused) continue;
                    if (toUpdateRrpvSet[way * shortSlots] > fusedVictimRrpv) {
                        fusedVictimWay = way;
                        fusedVictimRrpv = toUpdateRrpvSet[way * shortSlots];
                    }
                }
                // if has fused entry with higher RRPV, replace it;
                if (fusedVictimWay != numWays) {
                    stats.updateWriteFusedVictimFused++;
                    writeFusedEntry(toUpdateSet[fusedVictimWay]);
                    writtenSlot = &toUpdateSet[fusedVictimWay].shortSlots[0];
                    toUpdateRrpvSet[fusedVictimWay * shortSlots] = monitorMaxRrpv - 1;
                    toUpdateRrpvSet[fusedVictimWay * shortSlots + 1] = monitorMaxRrpv - 1;
                } else { // no fused entry, replace unfused entry which has the least sum of rrpv
                    stats.updateWriteFusedVictimUnfused++;
                    unsigned unfusedVictimWay = numWays;
                    unsigned unfusedVictimRrpvSum = 0;
                    for (unsigned way = 0; way < numWays; ++way) {
                        if (toUpdateSet[way].fused) continue;
                        unsigned rrpvSum = toUpdateRrpvSet[way * shortSlots] + \
                            toUpdateRrpvSet[way * shortSlots + 1];
                        if (rrpvSum > unfusedVictimRrpvSum) {
                            unfusedVictimWay = way;
                            unfusedVictimRrpvSum = rrpvSum;
                        }
                    }
                    writeFusedEntry(toUpdateSet[unfusedVictimWay]);
                    writtenSlot = &toUpdateSet[unfusedVictimWay].shortSlots[0];
                    toUpdateRrpvSet[unfusedVictimWay * shortSlots] = monitorMaxRrpv - 1;
                    toUpdateRrpvSet[unfusedVictimWay * shortSlots + 1] = monitorMaxRrpv - 1;
                }
            }
        } else { // write unfused entry with fused entry
            stats.updateWriteUnfusedOnFusedWay++;
            DPRINTF(BTBPDede,
                "BTBPDede: convert fused->unfused pc=%#lx victimWay=%u\n",
                pc, maxRrpvWay);
            toUpdateSet[maxRrpvWay].fused = false;
            toUpdateSet[maxRrpvWay].tag = monitorBTBTag;
            toUpdateSet[maxRrpvWay].shortSlots[0].valid = true;
            toUpdateSet[maxRrpvWay].shortSlots[0].bi = BranchInfo(entry);
            toUpdateSet[maxRrpvWay].shortSlots[0].bi.resolved = false;
            toUpdateSet[maxRrpvWay].shortSlots[1] = MonitorShortSlot();
            writtenSlot = &toUpdateSet[maxRrpvWay].shortSlots[0];

            std::transform(toUpdateRrpvSet.begin(), toUpdateRrpvSet.end(), toUpdateRrpvSet.begin(),
                [&](unsigned rrpv) { return rrpv + (monitorMaxRrpv - maxRrpv); });
            toUpdateRrpvSet[maxRrpvDistance] = monitorMaxRrpv - 1;
            toUpdateRrpvSet[maxRrpvDistanceBuddy] = monitorMaxRrpv;
        }
    } else { // hit
        if (foundSlot != shortSlots) { // hit short slot
            auto &slot = toUpdateSet[foundWay].shortSlots[foundSlot];
            slot.bi = BranchInfo(entry);
            slot.bi.resolved = false;
            writtenSlot = &slot;
            toUpdateRrpvSet[foundWay * shortSlots + foundSlot] = 0;
        } else { // hit long slot
            writeFusedEntry(toUpdateSet[foundWay]);
            writtenSlot = &toUpdateSet[foundWay].shortSlots[0];
            toUpdateRrpvSet[foundWay * shortSlots] = 0;
            toUpdateRrpvSet[foundWay * shortSlots + 1] = 0;
        }
    }

_counter_update:
    // counter update
    if (writtenSlot && entry.isCond) {
        int oldCtr = writtenSlot->ctr;
        bool oldAlwaysTaken = writtenSlot->alwaysTaken;
        if (foundWay == numWays) { // miss
            writtenSlot->alwaysTaken = thisBranchTaken;
            writtenSlot->ctr = thisBranchTaken ? 0 : -1;
        } else { // hit
            if (thisBranchTaken) {
                if (writtenSlot->ctr < 1) {
                    writtenSlot->ctr++;
                }
            } else {
                writtenSlot->alwaysTaken = false;
                if (writtenSlot->ctr > -2) {
                    writtenSlot->ctr--;
                }
            }
        }
        DPRINTF(BTBPDede,
            "BTBPDede: cond state update pc=%#lx taken=%d alwaysTaken %d->%d ctr %d->%d\n",
            pc, thisBranchTaken, oldAlwaysTaken, writtenSlot->alwaysTaken,
            oldCtr, writtenSlot->ctr);
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
       updateResolvedEntry(entry_to_update, stream);
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

}

void BTBPDede::printPageEntry(const PageEntry& e) {

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

    entry_to_write.tag = getMonitorBTBTag(entry_to_write.pc);
    stream.updateNewBTBEntry = entry_to_write;
    stream.updateIsOldEntry = is_old_entry;
}

void BTBPDede::commitBranch(const FetchTarget &stream, const DynInstPtr &inst) {
    auto meta = std::static_pointer_cast<BTBPDedeMeta>(stream.predMetas[getComponentIdx()]);
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
                stats.indirectMetaFound++;

                Addr predTarget = entry.target;
                bool predCrossPage = isCrossPage(pc, predTarget);
                if (predCrossPage) {
                    stats.indirectHitCrossPage++;
                } else {
                    stats.indirectHitNonCrossPage++;
                }

                if (predTarget == npc) {
                    stats.indirectPredCorrect++;
                } else {
                    stats.indirectPredWrong++;
                    if (predCrossPage) {
                        stats.indirectPredWrongCrossPage++;
                    } else {
                        stats.indirectPredWrongNonCrossPage++;
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
    ADD_STAT(updateLookupMiss, statistics::units::Count::get(),
        "update lookup misses in monitor BTB"),
    ADD_STAT(updateLookupMissNoAllocate, statistics::units::Count::get(),
        "update lookup misses that are not allocated"),
    ADD_STAT(updateLookupHitShortSlot, statistics::units::Count::get(),
        "update lookup hits on short slots"),
    ADD_STAT(updateLookupHitLongSlot, statistics::units::Count::get(),
        "update lookup hits on long slots"),
    ADD_STAT(updateWriteInvalidWay, statistics::units::Count::get(),
        "updates written into fully invalid way"),
    ADD_STAT(updateWritePartialInvalidSlot, statistics::units::Count::get(),
        "updates written into partial invalid slot"),
    ADD_STAT(updateWriteReplaceSameType, statistics::units::Count::get(),
        "updates replacing victim with same entry type"),
    ADD_STAT(updateWriteFuseOnUnfusedWay, statistics::units::Count::get(),
        "fused updates written on unfused way"),
    ADD_STAT(updateWriteUnfusedOnFusedWay, statistics::units::Count::get(),
        "unfused updates written on fused way"),
    ADD_STAT(updateWriteFusedVictimFused, statistics::units::Count::get(),
        "fused update chose fused victim way"),
    ADD_STAT(updateWriteFusedVictimUnfused, statistics::units::Count::get(),
        "fused update chose unfused victim way"),
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
