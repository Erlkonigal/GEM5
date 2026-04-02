#ifndef __CPU_PRED_BTB_PDEDE_HH__
#define __CPU_PRED_BTB_PDEDE_HH__

#include <cstdint>

#include "base/types.hh"
#include "cpu/pred/btb/common.hh"
#include "cpu/pred/btb/timed_base_pred.hh"

#ifdef UNIT_TEST
    #include "cpu/pred/btb/test/test_dprintf.hh"
#else
    #include "debug/BTBPDede.hh"
    #include "debug/PDedeStats.hh"
    #include "cpu/pred/general_arch_db.hh"
    #include "params/BTBPDede.hh"
    #include "sim/sim_object.hh"
#endif


namespace gem5 {
namespace branch_prediction {
namespace btb_pred {

class BTBPDede : public TimedBaseBTBPredictor
{
private:
    // Python parameters
    typedef BTBPDedeParams Params;
    unsigned instShiftAmt;
    unsigned numEntries;
    unsigned numPageEntries;
    unsigned numRegionEntries;
    unsigned numWays;
    unsigned numPageWays;
    unsigned numRegionWays;
    unsigned tagBits;
    unsigned tagFoldedBits;
    unsigned pageBits;
    unsigned victimCacheEntries;

    unsigned numAlignBanks; // fixed to 2 banks
    unsigned numSets;
    unsigned numPageSets;
    unsigned numRegionSets;
    unsigned numVictimCacheSets;

    static constexpr unsigned pageSize = 4096;

    static constexpr unsigned shortSlots = 2;
    static constexpr unsigned shortSlotTargetBits = 7;
    static constexpr unsigned longSlotTargetBits = 11;

    static constexpr unsigned monitorMaxRrpv = 3;
    static constexpr unsigned pageMaxRrpv = 3;
    static constexpr unsigned regionMaxRrpv = 3;

    struct MonitorShortSlot
    {
        bool valid = 0;
        bool alwaysTaken = true;
        BranchInfo bi;
        int ctr = 0;
    };

    struct MonitorLongSlot
    {
        // use slot0's valid
        // if both overflow and underflow are true, it means the target is cross page
        bool isCrossPage;
        bool isOverflow;
        bool isUnderflow;
        // pageBTB index and way
        Addr index;
        Addr way;
        BranchInfo bi;
    };

    struct MonitorEntry
    {
        bool fused = 0;
        Addr tag;
        MonitorShortSlot shortSlots[2];
        MonitorLongSlot longSlot;
    };
    typedef std::vector<MonitorEntry> MonitorSet;
    typedef std::vector<MonitorSet> MonitorAlignBank;

    struct PageEntry
    {
        bool valid;
        Addr vpnLower;
        // regionBTB way
        Addr way;
    };

    struct RegionEntry
    {
        bool valid;
        Addr vpnUpper;
    };
    typedef std::vector<PageEntry> PageSet;
    typedef std::vector<RegionEntry> RegionSet;

    struct BTBPDedeMeta
    {
        std::vector<BTBEntry> btbEntries;
        Addr startPC = 0;
    };

    std::shared_ptr<BTBPDedeMeta> meta;

#ifndef UNIT_TEST
    TraceManager *predTrace = nullptr;
    TraceManager *trainTrace = nullptr;
#endif

    // sram implementation
    std::vector<MonitorAlignBank> monitorBTB;
    std::vector<PageSet> pageBTB;
    std::vector<RegionSet> regionBTB;
    // std::vector<MonitorSet> victimCache;

    typedef std::vector<unsigned> RrpvSet;
    typedef std::vector<RrpvSet> RrpvBank;

    std::vector<RrpvBank> monitorRrpv;
    std::vector<RrpvSet> pageRrpv;
    std::vector<RrpvSet> regionRrpv;

    unsigned getPhysicalAlignBankIdx(Addr pc, unsigned logicBankIdx);
    Addr getShortSlotTarget(Addr pc, const MonitorShortSlot &slot, unsigned targetBits);
    Addr getLongSlotTarget(Addr pc, const MonitorLongSlot &slot, unsigned targetBits);

    bool isOverflow(Addr pc, Addr target, unsigned targetBits);
    bool isUnderflow(Addr pc, Addr target, unsigned targetBits);
    bool isCrossPage(Addr pc, Addr target);

    Addr getMonitorBTBIdx(Addr pc);
    Addr getMonitorBTBTag(Addr pc);
    std::vector<MonitorSet> getMonitorEntries(Addr pc);

    Addr getPageBTBIdx(Addr target, unsigned targetBits);
    Addr getVpnLower(Addr target, unsigned targetBits);
    Addr getVpnUpper(Addr target, unsigned targetBits);

    // Addr getVictimCacheTag(Addr monitorTag, Addr monitorIdx);

    std::vector<BTBEntry> processMonitorEntries(Addr pc, const std::vector<MonitorSet>& monitorSets);
    std::vector<BTBEntry> prepareUpdateEntries(const FetchTarget &stream);
    bool checkPredictionHit(const FetchTarget &stream, const BTBPDedeMeta *meta);
    void fillStagePredictions(
        const std::vector<BTBEntry>& btbEntries,
        std::vector<FullBTBPrediction>& stagePreds
    );

    void updateResolvedEntry(const BTBEntry &entry, const FetchTarget &stream,
                             bool predHit);

    void printBTBEntry(const BTBEntry& e);
    void dumpBTBEntries(const std::vector<BTBEntry>& es);

    void printMonitorEntry(const MonitorEntry& e);
    void printPageEntry(const PageEntry& e);
    void printRegionEntry(const RegionEntry& e);
    void dumpMonitorSetState(unsigned phyBankIdx, Addr alignedAddr, Addr monitorBTBIdx);
    void dumpLookupState(Addr pc);
    void dumpUpdateState(unsigned bankIdx, unsigned monitorBTBIdx,
                         unsigned pageBTBIdx, Addr vpnUpper);

#ifndef UNIT_TEST
    struct PDedePredTrace : public Record
    {
        void set(uint64_t startPC, uint64_t predTick, uint64_t logicBank,
                 uint64_t phyBank, uint64_t alignedAddr, uint64_t monitorIdx,
                 uint64_t monitorTag, uint64_t way, uint64_t slot,
                 uint64_t isLongSlot, uint64_t fused, uint64_t branchPC,
                 uint64_t target, uint64_t isCond, uint64_t isDirect,
                 uint64_t isIndirect, uint64_t isCall, uint64_t isReturn,
                 uint64_t alwaysTaken, int64_t ctr, uint64_t crossPage,
                 uint64_t overflow, uint64_t underflow, uint64_t pageIdx,
                 uint64_t pageWay, uint64_t regionWay, uint64_t hit)
        {
            _tick = curTick();
            _uint64_data["startPC"] = startPC;
            _uint64_data["predTick"] = predTick;
            _uint64_data["logicBank"] = logicBank;
            _uint64_data["phyBank"] = phyBank;
            _uint64_data["alignedAddr"] = alignedAddr;
            _uint64_data["monitorIdx"] = monitorIdx;
            _uint64_data["monitorTag"] = monitorTag;
            _uint64_data["way"] = way;
            _uint64_data["slot"] = slot;
            _uint64_data["isLongSlot"] = isLongSlot;
            _uint64_data["fused"] = fused;
            _uint64_data["branchPC"] = branchPC;
            _uint64_data["target"] = target;
            _uint64_data["isCond"] = isCond;
            _uint64_data["isDirect"] = isDirect;
            _uint64_data["isIndirect"] = isIndirect;
            _uint64_data["isCall"] = isCall;
            _uint64_data["isReturn"] = isReturn;
            _uint64_data["alwaysTaken"] = alwaysTaken;
            _uint64_data["ctr"] = static_cast<uint64_t>(ctr);
            _uint64_data["crossPage"] = crossPage;
            _uint64_data["overflow"] = overflow;
            _uint64_data["underflow"] = underflow;
            _uint64_data["pageIdx"] = pageIdx;
            _uint64_data["pageWay"] = pageWay;
            _uint64_data["regionWay"] = regionWay;
            _uint64_data["hit"] = hit;
        }
    };

    struct PDedeTrainTrace : public Record
    {
        void set(uint64_t startPC, uint64_t exePC, uint64_t controlPC,
                 uint64_t target, uint64_t taken, uint64_t mispredict,
                 uint64_t predHit, uint64_t dist, uint64_t canUseShortSlot,
                 uint64_t updateIsFused, uint64_t bankIdx, uint64_t monitorIdx,
                 uint64_t monitorTag, uint64_t pageIdx, uint64_t vpnLower,
                 uint64_t vpnUpper, uint64_t foundWay, uint64_t foundSlot,
                 uint64_t lookupHitWay, uint64_t lookupHitShortSlot,
                 uint64_t lookupHitLongSlot, uint64_t lookupMiss,
                 uint64_t chooseInvalidWay, uint64_t choosePartialInvalidSlot,
                 uint64_t replaceSameType, uint64_t fuseOnUnfusedWay,
                 uint64_t unfusedOnFusedWay, uint64_t fusedVictimFused,
                 uint64_t fusedVictimUnfused, uint64_t allocPageEntry,
                 uint64_t allocRegionEntry, uint64_t reusePageEntry,
                 uint64_t reuseRegionEntry, uint64_t counterUpdate,
                 uint64_t finalWay, uint64_t finalSlot, uint64_t finalFused,
                 uint64_t finalCrossPage, uint64_t finalPageIdx,
                 uint64_t finalPageWay, uint64_t finalRegionWay,
                 uint64_t oldAlwaysTaken, uint64_t newAlwaysTaken,
                 int64_t oldCtr, int64_t newCtr, uint64_t writeSuccess)
        {
            _tick = curTick();
            _uint64_data["startPC"] = startPC;
            _uint64_data["exePC"] = exePC;
            _uint64_data["controlPC"] = controlPC;
            _uint64_data["target"] = target;
            _uint64_data["taken"] = taken;
            _uint64_data["mispredict"] = mispredict;
            _uint64_data["predHit"] = predHit;
            _uint64_data["dist"] = dist;
            _uint64_data["canUseShortSlot"] = canUseShortSlot;
            _uint64_data["updateIsFused"] = updateIsFused;
            _uint64_data["bankIdx"] = bankIdx;
            _uint64_data["monitorIdx"] = monitorIdx;
            _uint64_data["monitorTag"] = monitorTag;
            _uint64_data["pageIdx"] = pageIdx;
            _uint64_data["vpnLower"] = vpnLower;
            _uint64_data["vpnUpper"] = vpnUpper;
            _uint64_data["foundWay"] = foundWay;
            _uint64_data["foundSlot"] = foundSlot;
            _uint64_data["lookupHitWay"] = lookupHitWay;
            _uint64_data["lookupHitShortSlot"] = lookupHitShortSlot;
            _uint64_data["lookupHitLongSlot"] = lookupHitLongSlot;
            _uint64_data["lookupMiss"] = lookupMiss;
            _uint64_data["chooseInvalidWay"] = chooseInvalidWay;
            _uint64_data["choosePartialInvalidSlot"] = choosePartialInvalidSlot;
            _uint64_data["replaceSameType"] = replaceSameType;
            _uint64_data["fuseOnUnfusedWay"] = fuseOnUnfusedWay;
            _uint64_data["unfusedOnFusedWay"] = unfusedOnFusedWay;
            _uint64_data["fusedVictimFused"] = fusedVictimFused;
            _uint64_data["fusedVictimUnfused"] = fusedVictimUnfused;
            _uint64_data["allocPageEntry"] = allocPageEntry;
            _uint64_data["allocRegionEntry"] = allocRegionEntry;
            _uint64_data["reusePageEntry"] = reusePageEntry;
            _uint64_data["reuseRegionEntry"] = reuseRegionEntry;
            _uint64_data["counterUpdate"] = counterUpdate;
            _uint64_data["finalWay"] = finalWay;
            _uint64_data["finalSlot"] = finalSlot;
            _uint64_data["finalFused"] = finalFused;
            _uint64_data["finalCrossPage"] = finalCrossPage;
            _uint64_data["finalPageIdx"] = finalPageIdx;
            _uint64_data["finalPageWay"] = finalPageWay;
            _uint64_data["finalRegionWay"] = finalRegionWay;
            _uint64_data["oldAlwaysTaken"] = oldAlwaysTaken;
            _uint64_data["newAlwaysTaken"] = newAlwaysTaken;
            _uint64_data["oldCtr"] = static_cast<uint64_t>(oldCtr);
            _uint64_data["newCtr"] = static_cast<uint64_t>(newCtr);
            _uint64_data["writeSuccess"] = writeSuccess;
        }
    };
#endif

    typedef statistics::Scalar Scalar;
    struct PDedeStats : public statistics::Group
    {
        Scalar newEntry;
        Scalar newEntryWithCond;
        Scalar newEntryWithUncond;

        Scalar predTimes;
        Scalar predMissTimes;
        Scalar predHitTimes;
        Scalar predHitEntries;
        Scalar predHitVictimTimes;
        Scalar predHitVictimEntries;

        Scalar updateTimes;
        Scalar updateMiss;
        Scalar updateHit;

        Scalar updateLookupMiss;
        Scalar updateLookupMissNoAllocate;
        Scalar updateLookupHitShortSlot;
        Scalar updateLookupHitLongSlot;

        Scalar updateWriteInvalidWay;
        Scalar updateWritePartialInvalidSlot;
        Scalar updateWriteReplaceSameType;
        Scalar updateWriteFuseOnUnfusedWay;
        Scalar updateWriteUnfusedOnFusedWay;
        Scalar updateWriteFusedVictimFused;
        Scalar updateWriteFusedVictimUnfused;

        Scalar allBranchHits;
        Scalar totalBranchHits;
        Scalar allBranchHitTakens;
        Scalar allBranchHitNotTakens;
        Scalar allBranchMisses;
        Scalar totalBranchMisses;
        Scalar allBranchMissTakens;
        Scalar allBranchMissNotTakens;

        Scalar condHits;
        Scalar condHitTakens;
        Scalar condHitNotTakens;
        Scalar condMisses;
        Scalar condMissTakens;
        Scalar condMissNotTakens;
        Scalar condPredCorrect;
        Scalar condPredWrong;

        Scalar uncondHits;
        Scalar uncondMisses;

        Scalar indirectHits;
        Scalar indirectMisses;
        Scalar indirectPredCorrect;
        Scalar indirectPredWrong;
        Scalar indirectMetaFound;
        Scalar indirectMetaNotFound;
        Scalar indirectHitCrossPage;
        Scalar indirectHitNonCrossPage;
        Scalar indirectPredWrongCrossPage;
        Scalar indirectPredWrongNonCrossPage;

        Scalar callHits;
        Scalar callMisses;

        Scalar returnHits;
        Scalar returnMisses;

        // statistics::Distribution condTargetDiffBits;
        // statistics::Distribution uncondTargetDiffBits;
        // statistics::Distribution indirectTargetDiffBits;
        // statistics::Distribution callTargetDiffBits;
        // statistics::Distribution returnTargetDiffBits;

        // statistics::Distribution condAllocPartitionIdx;
        // statistics::Distribution uncondAllocPartitionIdx;
        // statistics::Distribution indirectAllocPartitionIdx;
        // statistics::Distribution callAllocPartitionIdx;
        // statistics::Distribution returnAllocPartitionIdx;

        PDedeStats(statistics::Group* parent);
    } stats;

public:
    BTBPDede(const Params& p);
    ~BTBPDede() override;

    void putPCHistory(Addr startAddr,
                      const boost::dynamic_bitset<>& history,
                      std::vector<FullBTBPrediction>& stagePreds) override;

    std::shared_ptr<void> getPredictionMeta() override;

    void update(const FetchTarget& stream) override;

#ifndef UNIT_TEST
    void setTrace() override;
#endif

    void commitBranch(const FetchTarget &stream, const DynInstPtr &inst) override;

    void getAndSetNewBTBEntry(FetchTarget &stream);
};

}
}
}


#endif // __CPU_PRED_BTB_PDEDE_HH__
