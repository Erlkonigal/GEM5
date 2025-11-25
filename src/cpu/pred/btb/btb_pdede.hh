#ifndef __CPU_PRED_BTB_PDEDE_HH__
#define __CPU_PRED_BTB_PDEDE_HH__

#include "cpu/pred/btb/stream_struct.hh"
#include "cpu/pred/btb/timed_base_pred.hh"

#ifdef UNIT_TEST
    #include "cpu/pred/btb/test/test_dprintf.hh"
#else
    #include "params/BTBPDEDE.hh"
    #include "sim/sim_object.hh"
#endif


namespace gem5 {
namespace branch_prediction {
namespace btb_pred {

class BTBPDEDE : public TimedBaseBTBPredictor
{
private:
    // Python parameters
    typedef BTBPDEDEParams Params;
    unsigned instShiftAmt;
    unsigned numEntries;
    unsigned numWays;
    unsigned tagBits;
    unsigned pageBits;

    unsigned numAlignBanks; // fixed to 2 banks

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

    struct MonitorEntry
    {
        bool valid;
        int ctr; // not used currently
        int useful;
        Addr tag;
        Addr offset;
        Addr pagePointer;
        BranchAttribute attr;
        // const attributes
        const int offsetBits;
        MonitorEntry(int offsetBits)
            : valid(false), ctr(0), useful(0),
              tag(0), offset(0), pagePointer(0),
              attr({
                BranchAttribute::BranchTypeEnum::None,
                BranchAttribute::RasActionEnum::None
              }),
              offsetBits(offsetBits) {}
    };
    typedef std::vector<MonitorEntry> MonitorSet;
    typedef std::vector<MonitorSet> MonitorAlignBank;
    std::vector<MonitorAlignBank> monitorTable;

    struct PageEntry
    {
        Addr offset;
        int ctr;
    };
    typedef PageEntry PageSet;
    std::vector<PageSet> pageTable;

    struct RegionEntry
    {
        Addr offset;
        int ctr;
    };
    typedef RegionEntry RegionSet;
    std::vector<RegionSet> regionTable;

public:
    BTBPDEDE(const Params& p);
    ~BTBPDEDE() override;

    void tickStart() override;

    void tick() override;

    void putPCHistory(Addr startAddr,
                      const boost::dynamic_bitset<>& history,
                      std::vector<FullBTBPrediction>& stagePreds) override;

    std::shared_ptr<void> getPredictionMeta() override;

    void specUpdateHist(const boost::dynamic_bitset<>& history,
                        FullBTBPrediction& pred) override;

    void recoverHist(const boost::dynamic_bitset<>& history,
                     const FetchStream& entry,
                     int shamt,
                     bool cond_taken) override;

    void update(const FetchStream& entry) override;
};

}
}
}


#endif // __CPU_PRED_BTB_PDEDE_HH__
