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
        bool alwaysTaken;
        BranchInfo bi;
        int ctr;
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
        std::vector<MonitorSet> rawMonitorSets;
        std::vector<BTBEntry> btbEntries;
    };

    std::shared_ptr<BTBPDedeMeta> meta;

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
    void checkPredictionHit(const FetchTarget &stream, const BTBPDedeMeta *meta);
    void fillStagePredictions(
        const std::vector<BTBEntry>& btbEntries,
        std::vector<FullBTBPrediction>& stagePreds
    );

    void updateResolvedEntry(const BTBEntry &entry, const FetchTarget &stream);

    void printBTBEntry(const BTBEntry& e);
    void dumpBTBEntries(const std::vector<BTBEntry>& es);

    void printMonitorEntry(const MonitorEntry& e);
    void printPageEntry(const PageEntry& e);
    void printRegionEntry(const RegionEntry& e);

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
        Scalar updateMissTimes;
        Scalar updateFoundEmptyTimes;
        Scalar updateEvictTimes;

        Scalar updateHitTimes;
        Scalar updateHitVictimTimes;
        Scalar updateMultiHitTimes;
        Scalar updateExisting;
        Scalar updateReplace;
        Scalar updateReplaceValidOne;
        Scalar updateInVC;
        Scalar updateTotal;
        Scalar updateFixTarget;

        Scalar updateUsePagePointerTimes;
        Scalar updateAllocatePagePointerTimes;
        Scalar updateNotUseButHasPagePointerTimes;
        Scalar updateNotUseAndNoPagePointerTimes;

        Scalar carryOverflowTimes;

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
        Scalar indirectPredWrongCarryFit;
        Scalar indirectPredWrongCarryPlusOne;
        Scalar indirectPredWrongCarryMinusOne;
        Scalar indirectPredWrongCarryNone;
        Scalar indirectPredWrongUsePagePointer;
        Scalar indirectPredWrongNoPagePointer;

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

    void commitBranch(const FetchTarget &stream, const DynInstPtr &inst) override;

    void getAndSetNewBTBEntry(FetchTarget &stream);
};

}
}
}


#endif // __CPU_PRED_BTB_PDEDE_HH__
