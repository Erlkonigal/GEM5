#ifndef __CPU_PRED_BTB_PDEDE_HH__
#define __CPU_PRED_BTB_PDEDE_HH__

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

    // TODO: offset bits need profiling to decide
    static constexpr unsigned maxOffsetBits = 11;

    struct BranchAttribute
    {
        enum class BranchTypeEnum
        {
            None, Conditional, Direct, Indirect
        } branchType;
        enum class RasActionEnum
        {
            None, Pop, Push, PopAndPush
        } rasAction;
    };
    struct TargetCarry
    {
        enum class TargetCarryEnum
        {
            Fit, PlusOne, MinusOne, None
        } targetCarry;

        bool isFit() const {
            return targetCarry == TargetCarryEnum::Fit;
        }
        bool isPlusOne() const {
            return targetCarry == TargetCarryEnum::PlusOne;
        }
        bool isMinusOne() const {
            return targetCarry == TargetCarryEnum::MinusOne;
        }
        bool isNone() const {
            return targetCarry == TargetCarryEnum::None;
        }
    };

    struct MonitorEntry
    {
    private:
        // const attributes
        int offsetBits;
        bool usePagePointer;
    public:
        bool valid;
        bool isCrossPage;
        bool alwaysTaken;
        unsigned position; // position in the fetch block
        Addr tag;
        Addr targetOffset; // target low bits
        BranchAttribute attr;
        TargetCarry carry;

        Addr pageTableSet;
        union ExtInfo
        {
            Addr pageTableWay; // used as long target with page pointer
            int8_t ctr;        // used as conditional short target prediction counter, range [-2, 1]
        } extendedInfo;


        MonitorEntry() : offsetBits(maxOffsetBits), usePagePointer(true), valid(false),
            isCrossPage(false), alwaysTaken(true) {}
        MonitorEntry(int offsetBits, bool usePagePointer)
            : offsetBits(offsetBits), usePagePointer(usePagePointer), valid(false),
              isCrossPage(false), alwaysTaken(true) {}

        int getOffsetBits() const { return offsetBits; }
        bool isUsePagePointer() const { return usePagePointer; }
    };
    typedef std::vector<MonitorEntry> MonitorSet;
    typedef std::vector<MonitorSet> MonitorAlignBank;

    struct PageEntry
    {
        Addr tag;
        int ctr;
    };
    typedef std::vector<PageEntry> PageSet;

    struct RegionEntry
    {
        Addr tag;
        int ctr;
    };
    typedef std::vector<RegionEntry> RegionSet;

    struct BTBPDedeMeta
    {
        std::vector<MonitorSet> rawMonitorSets;
        std::vector<BTBEntry> btbEntries;
    };
    // typedef std::vector<MonitorSet> BTBPDedeMeta;

    typedef std::vector<unsigned> ReplacementAlignBank;

    std::shared_ptr<BTBPDedeMeta> meta;

    // sram implementation
    std::vector<MonitorAlignBank> monitorTable;

    // register implementation
    std::vector<ReplacementAlignBank> monitorPLRUTable; // PLRU replacement table

    std::vector<PageSet> pageTable;
    std::vector<unsigned> pagePLRUTable;    // page table PLRU replacement table

    std::vector<MonitorSet> victimCache;
    std::vector<unsigned> victimCachePLRUTable;
    // std::vector<RegionSet> regionTable; // TODO: implement region table

    unsigned getRotatedAlignBankIdx(Addr pc, unsigned logicBankIdx);
    Addr getFullTarget(Addr pc, const MonitorEntry &entry);
    TargetCarry computeCarryBits(Addr pc, Addr target, unsigned offsetBits);

    std::vector<unsigned> getPLRUVictims(unsigned state, unsigned numWays);
    unsigned getTouchedPLRUState(unsigned state, unsigned numWays, unsigned touchWay);
    unsigned getMakeVictimPLRUState(unsigned state, unsigned numWays, unsigned victimWay);

    Addr getMonitorIdx(Addr pc);
    Addr getMonitorTag(Addr pc);
    std::vector<MonitorSet> getMonitorEntries(Addr pc);

    Addr getPageTableIdx(Addr target);
    Addr getPageTableTag(Addr target);

    Addr getVictimCacheTag(Addr monitorTag, Addr monitorIdx);

    std::vector<BTBEntry> processMonitorEntries(Addr pc, const std::vector<MonitorSet>& monitorSets);
    void fillStagePredictions(
        const std::vector<BTBEntry>& btbEntries,
        std::vector<FullBTBPrediction>& stagePreds
    );

    // unsigned getTargetDiffBits(Addr pc, Addr target);
    // unsigned getPartitionIdx(const BranchInfo &exec, const MonitorSet &meta);

    void printBTBEntry(const BTBEntry& e);
    void dumpBTBEntries(const std::vector<BTBEntry>& es);

    void printMonitorEntry(const MonitorEntry& e);
    void printPageEntry(const PageEntry& e);

    typedef statistics::Scalar Scalar;
    struct PDedeStats : public statistics::Group
    {
        Scalar predTimes;
        Scalar predMissTimes;
        Scalar predHitTimes;
        Scalar predHitEntries;
        Scalar predHitVictimTimes;
        Scalar predHitVictimEntries;

        Scalar updateTimes;
        Scalar updateMissTimes;
        Scalar updateFoundEmptyTimes;
        Scalar updateEvictTimes;

        Scalar updateHitTimes;
        Scalar updateHitVictimTimes;
        Scalar updateMultiHitTimes;

        Scalar updateUsePagePointerTimes;
        Scalar updateAllocatePagePointerTimes;
        Scalar updateNotUseButHasPagePointerTimes;
        Scalar updateNotUseAndNoPagePointerTimes;

        Scalar carryOverflowTimes;

        Scalar totalBranchHits;
        Scalar totalBranchMisses;

        Scalar condHits;
        Scalar condMisses;

        Scalar uncondHits;
        Scalar uncondMisses;

        Scalar indirectHits;
        Scalar indirectMisses;

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
};

}
}
}


#endif // __CPU_PRED_BTB_PDEDE_HH__
