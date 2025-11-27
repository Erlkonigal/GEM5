#ifndef __CPU_PRED_BTB_PDEDE_HH__
#define __CPU_PRED_BTB_PDEDE_HH__

#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/timed_base_pred.hh"

#ifdef UNIT_TEST
    #include "cpu/pred/btb/test/test_dprintf.hh"
#else
    #include "debug/BTBPDede.hh"
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

    unsigned numAlignBanks; // fixed to 2 banks
    unsigned numSets;
    unsigned numPageSets;
    unsigned numRegionSets;

    // TODO: offset bits need profiling to decide
    const unsigned maxOffsetBits = 11;

    const std::vector<unsigned> way4OffsetBits = {4, 7, 11, maxOffsetBits};
    const std::vector<bool> way4UsePagePointer = {false, false, false, true};

    const std::vector<unsigned> way8OffsetBits = {0, 4, 5, 7, 9, 11, maxOffsetBits, maxOffsetBits};
    const std::vector<bool> way8UsePagePointer = {false, false, false, false,
                                                  false, false, true, true};

    std::vector<unsigned> wayOffsetBits;
    std::vector<bool> wayUsePagePointer;

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
            Fit, PlusOne, MinusOne
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
    };

    struct MonitorEntry
    {
        bool valid;
        bool isCrossPage;
        unsigned position; // position in the fetch block
        Addr tag;
        Addr targetOffset; // target low bits
        Addr pagePointerSet;
        Addr pagePointerWay;
        TargetCarry carry;
        BranchAttribute attr;
        // const attributes
        const int offsetBits;
        const bool usePagePointer;
        MonitorEntry(int offsetBits, bool usePagePointer)
            : offsetBits(offsetBits), usePagePointer(usePagePointer) {}
    };
    typedef std::vector<MonitorEntry> MonitorSet;
    typedef std::vector<MonitorSet> MonitorAlignBank;

    struct PageEntry
    {
        TargetCarry carry;
        Addr tag;
        int ctr;
    };
    typedef std::vector<PageEntry> PageSet;

    struct RegionEntry
    {
        TargetCarry carry;
        Addr tag;
        int ctr;
    };
    typedef std::vector<RegionEntry> RegionSet;

    struct BTBPDedeMeta
    {
        std::vector<MonitorSet> rawMonitorSets;
    };

    typedef std::vector<unsigned> ReplacementAlignBank;

    // register implementation
    std::vector<ReplacementAlignBank> monitorPLRUTable; // PLRU replacement table
    std::vector<unsigned> pagePLRUTable;    // page table PLRU replacement table
    std::vector<PageSet> pageTable;
    // std::vector<RegionSet> regionTable; // TODO: implement region table

    // sram implementation
    std::vector<MonitorAlignBank> monitorTable;
    std::shared_ptr<BTBPDedeMeta> meta;

    unsigned getRotatedAlignBankIdx(Addr pc, unsigned logicBankIdx);
    Addr getFullTarget(Addr pc, const MonitorEntry &entry);
    TargetCarry computeCarryBits(Addr pc, Addr target, unsigned offsetBits);

    std::vector<unsigned> getPLRUVictims(unsigned state, unsigned numWays);
    unsigned getUpdatedPLRUState(unsigned state, unsigned numWays, unsigned touchWay);

    Addr getMonitorIdx(Addr pc);
    Addr getMonitorTag(Addr pc);
    std::vector<MonitorSet> getMonitorEntries(Addr pc);

    Addr getPageTableIdx(Addr pc);
    Addr getPageTableTag(Addr pc);

    std::vector<BTBEntry> processMonitorEntries(Addr pc, const std::vector<MonitorSet>& monitorSets);
    void fillStagePredictions(
        const std::vector<BTBEntry>& btbEntries,
        std::vector<FullBTBPrediction>& stagePreds
    );

    unsigned getTargetDiffBits(Addr pc, Addr target);
    unsigned getPartitionIdx(const BranchInfo &exec, std::shared_ptr<BTBPDedeMeta> meta);

    void printBTBEntry(const BTBEntry& e);
    void dumpBTBEntries(const std::vector<BTBEntry>& es);

public:
    BTBPDede(const Params& p);
    ~BTBPDede() override;

    void putPCHistory(Addr startAddr,
                      const boost::dynamic_bitset<>& history,
                      std::vector<FullBTBPrediction>& stagePreds) override;

    std::shared_ptr<void> getPredictionMeta() override;

    void update(const FetchStream& stream) override;
};

}
}
}


#endif // __CPU_PRED_BTB_PDEDE_HH__
