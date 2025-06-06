/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#define NS_LOG_APPEND_CONTEXT                                                                      \
    do                                                                                             \
    {                                                                                              \
        std::clog << " [ CellId " << GetCellId() << ", bwpId " << GetBwpId() << "] ";              \
    } while (false);

#include "nr-phy.h"

#include "beam-manager.h"
#include "nr-net-device.h"
#include "nr-spectrum-phy.h"

#include "ns3/pointer.h"
#include "ns3/uniform-planar-array.h"
#include <ns3/boolean.h>

#include <algorithm>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrPhy");

NS_OBJECT_ENSURE_REGISTERED(NrPhy);

/*   SAP   */
class NrMemberPhySapProvider : public NrPhySapProvider
{
  public:
    NrMemberPhySapProvider(NrPhy* phy);

    void SendMacPdu(const Ptr<Packet>& p,
                    const SfnSf& sfn,
                    uint8_t symStart,
                    uint16_t rnti) override;

    void SendControlMessage(Ptr<NrControlMessage> msg) override;

    void SendRachPreamble(uint8_t PreambleId, uint8_t Rnti) override;

    void SetSlotAllocInfo(const SlotAllocInfo& slotAllocInfo) override;

    BeamId GetBeamId(uint8_t rnti) const override;

    Ptr<const SpectrumModel> GetSpectrumModel() override;

    void NotifyConnectionSuccessful() override;

    uint16_t GetBwpId() const override;

    uint16_t GetCellId() const override;

    uint32_t GetSymbolsPerSlot() const override;

    Time GetSlotPeriod() const override;

    uint32_t GetRbNum() const override;

  private:
    NrPhy* m_phy;
};

NrMemberPhySapProvider::NrMemberPhySapProvider(NrPhy* phy)
    : m_phy(phy)
{
    //  Nothing more to do
}

void
NrMemberPhySapProvider::SendMacPdu(const Ptr<Packet>& p,
                                   const SfnSf& sfn,
                                   uint8_t symStart,
                                   uint16_t rnti)
{
    m_phy->SetMacPdu(p, sfn, symStart, rnti);
}

void
NrMemberPhySapProvider::SendControlMessage(Ptr<NrControlMessage> msg)
{
    m_phy->EnqueueCtrlMessage(msg); // May need to change
}

void
NrMemberPhySapProvider::SendRachPreamble(uint8_t PreambleId, uint8_t Rnti)
{
    m_phy->SendRachPreamble(PreambleId, Rnti);
}

void
NrMemberPhySapProvider::SetSlotAllocInfo(const SlotAllocInfo& slotAllocInfo)
{
    m_phy->PushBackSlotAllocInfo(slotAllocInfo);
}

BeamId
NrMemberPhySapProvider::GetBeamId(uint8_t rnti) const
{
    return m_phy->GetBeamId(rnti);
}

Ptr<const SpectrumModel>
NrMemberPhySapProvider::GetSpectrumModel()
{
    return m_phy->GetSpectrumModel();
}

void
NrMemberPhySapProvider::NotifyConnectionSuccessful()
{
    m_phy->NotifyConnectionSuccessful();
}

uint16_t
NrMemberPhySapProvider::GetBwpId() const
{
    return m_phy->GetBwpId();
}

uint16_t
NrMemberPhySapProvider::GetCellId() const
{
    return m_phy->GetCellId();
}

uint32_t
NrMemberPhySapProvider::GetSymbolsPerSlot() const
{
    return m_phy->GetSymbolsPerSlot();
}

Time
NrMemberPhySapProvider::GetSlotPeriod() const
{
    return m_phy->GetSlotPeriod();
}

uint32_t
NrMemberPhySapProvider::GetRbNum() const
{
    return m_phy->GetRbNum();
}

/* ======= */

TypeId
NrPhy::GetTypeId()
{
    static TypeId tid = TypeId("ns3::NrPhy")
                            .SetParent<Object>()
                            .AddAttribute("NrSpectrumPhy",
                                          "NrSpectrumPhy instance",
                                          PointerValue(),
                                          MakePointerAccessor(&NrPhy::m_spectrumPhy),
                                          MakePointerChecker<NrSpectrumPhy>());

    return tid;
}

std::vector<int>
NrPhy::FromRBGBitmaskToRBAssignment(const std::vector<uint8_t> rbgBitmask) const
{
    std::vector<int> ret;

    for (uint32_t i = 0; i < rbgBitmask.size(); ++i)
    {
        if (rbgBitmask.at(i) == 1)
        {
            for (uint32_t k = 0; k < GetNumRbPerRbg(); ++k)
            {
                ret.push_back((i * GetNumRbPerRbg()) + k);
            }
        }
    }

    NS_ASSERT(static_cast<uint32_t>(std::count(rbgBitmask.begin(), rbgBitmask.end(), 1) *
                                    GetNumRbPerRbg()) == ret.size());
    return ret;
}

NrPhy::NrPhy()
    : m_currSlotAllocInfo(SfnSf(0, 0, 0, 0)),
      m_tbDecodeLatencyUs(100.0)
{
    NS_LOG_FUNCTION(this);
    m_phySapProvider = new NrMemberPhySapProvider(this);
    m_nrSlUePhySapProvider = new MemberNrSlUePhySapProvider<NrPhy>(this);
}

NrPhy::~NrPhy()
{
    NS_LOG_FUNCTION(this);
}

void
NrPhy::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_slotAllocInfo.clear();
    m_controlMessageQueue.clear();
    m_packetBurstMap.clear();
    m_ctrlMsgs.clear();
    m_tddPattern.clear();
    m_netDevice = nullptr;
    if (m_spectrumPhy)
    {
        m_spectrumPhy->Dispose();
    }
    m_spectrumPhy = nullptr;
    delete m_phySapProvider;
    delete m_nrSlUePhySapProvider;
}

void
NrPhy::SetDevice(Ptr<NrNetDevice> d)
{
    NS_LOG_FUNCTION(this);
    m_netDevice = d;
}

void
NrPhy::InstallCentralFrequency(double f)
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_IF(m_centralFrequency >= 0.0);
    m_centralFrequency = f;
}

void
NrPhy::SetChannelBandwidth(uint16_t channelBandwidth)
{
    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("SetChannelBandwidth called with channel bandwidth value: "
                 << channelBandwidth * 100 * 1000
                 << "Hz, "
                    "and the previous value of channel bandwidth was: "
                 << GetChannelBandwidth() << " Hz");

    if (m_channelBandwidth != channelBandwidth)
    {
        m_channelBandwidth = channelBandwidth;
        // number of RB and noise PSD must be updated when bandwidth or numerology gets changed
        DoUpdateRbNum();
    }
}

void
NrPhy::SetNumerology(uint16_t numerology)
{
    NS_LOG_FUNCTION(numerology);
    m_numerology = numerology;
    m_slotsPerSubframe = static_cast<uint16_t>(std::pow(2, numerology));
    m_slotPeriod = Seconds(0.001 / m_slotsPerSubframe);
    m_subcarrierSpacing = 15000 * static_cast<uint32_t>(std::pow(2, numerology));
    m_symbolPeriod = (m_slotPeriod / m_symbolsPerSlot);

    // number of RB and noise PSD must be updated when bandwidth or numerology gets changed
    if (m_channelBandwidth != 0)
    {
        DoUpdateRbNum();

        NS_LOG_INFO(" Numerology configured:"
                    << GetNumerology() << " slots per subframe: " << m_slotsPerSubframe
                    << " slot period:" << GetSlotPeriod() << " symbol period:" << GetSymbolPeriod()
                    << " subcarrier spacing: " << GetSubcarrierSpacing()
                    << " number of RBs: " << GetRbNum());
    }
    else
    {
        NS_LOG_DEBUG("Numerology is set, but the channel bandwidth not yet, so the number of RBs "
                     "cannot be updated now.");
    }
}

uint16_t
NrPhy::GetNumerology() const
{
    return m_numerology;
}

void
NrPhy::SetSymbolsPerSlot(uint16_t symbolsPerSlot)
{
    NS_LOG_FUNCTION(this);
    m_symbolsPerSlot = symbolsPerSlot;
    m_symbolPeriod = (m_slotPeriod / m_symbolsPerSlot);
}

void
NrPhy::SetRbOverhead(double oh)
{
    m_rbOh = oh;
}

double
NrPhy::GetRbOverhead() const
{
    return m_rbOh;
}

uint32_t
NrPhy::GetSymbolsPerSlot() const
{
    return m_symbolsPerSlot;
}

Time
NrPhy::GetSlotPeriod() const
{
    NS_ABORT_IF(m_slotPeriod.IsNegative());
    return m_slotPeriod;
}

void
NrPhy::DoSetCellId(uint16_t cellId)
{
    NS_LOG_FUNCTION(this);
    m_cellId = cellId;
}

void
NrPhy::SendRachPreamble(uint32_t PreambleId, uint32_t Rnti)
{
    NS_LOG_FUNCTION(this);
    m_raPreambleId = PreambleId;
    Ptr<NrRachPreambleMessage> msg = Create<NrRachPreambleMessage>();
    msg->SetSourceBwp(GetBwpId());
    msg->SetRapId(PreambleId);
    EnqueueCtrlMsgNow(msg);
}

void
NrPhy::SetMacPdu(const Ptr<Packet>& p, const SfnSf& sfn, uint8_t symStart, uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(sfn.GetNumerology() == GetNumerology());
    uint64_t key = sfn.GetEncodingWithSymStartRnti(symStart, rnti);
    auto it = m_packetBurstMap.find(key);

    if (it == m_packetBurstMap.end())
    {
        it = m_packetBurstMap.insert(std::make_pair(key, CreateObject<PacketBurst>())).first;
    }
    it->second->AddPacket(p);
    NS_LOG_INFO("Adding a packet for the Packet Burst of " << sfn << " at sym " << +symStart
                                                           << std::endl);
}

void
NrPhy::NotifyConnectionSuccessful()
{
    NS_LOG_FUNCTION(this);
}

Ptr<PacketBurst>
NrPhy::GetPacketBurst(SfnSf sfn, uint8_t sym, uint16_t rnti)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(sfn.GetNumerology() == GetNumerology());
    Ptr<PacketBurst> pburst;
    auto it = m_packetBurstMap.find(sfn.GetEncodingWithSymStartRnti(sym, rnti));

    if (it == m_packetBurstMap.end())
    {
        // For instance, this can happen with low BW and low MCS: The MAC
        // ignores the txOpportunity.
        NS_LOG_WARN("Packet burst not found for " << sfn << " at sym " << +sym);
        return pburst;
    }
    else
    {
        pburst = it->second;
        m_packetBurstMap.erase(it);
    }
    return pburst;
}

Ptr<SpectrumValue>
NrPhy::GetNoisePowerSpectralDensity()
{
    return NrSpectrumValueHelper::CreateNoisePowerSpectralDensity(m_noiseFigure,
                                                                  GetSpectrumModel());
}

Ptr<SpectrumValue>
NrPhy::GetTxPowerSpectralDensity(const std::vector<int>& rbIndexVector)
{
    Ptr<const SpectrumModel> sm = GetSpectrumModel();

    return NrSpectrumValueHelper::CreateTxPowerSpectralDensity(m_txPower,
                                                               rbIndexVector,
                                                               sm,
                                                               m_powerAllocationType);
}

double
NrPhy::GetCentralFrequency() const
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_IF(m_centralFrequency < 0.0);
    return m_centralFrequency;
}

std::string
NrPhy::GetPattern(const std::vector<LteNrTddSlotType>& pattern)
{
    static std::unordered_map<LteNrTddSlotType, std::string, std::hash<int>> lookupTable = {
        {LteNrTddSlotType::DL, "DL"},
        {LteNrTddSlotType::UL, "UL"},
        {LteNrTddSlotType::S, "S"},
        {LteNrTddSlotType::F, "F"}};

    std::stringstream ss;

    for (const auto& v : pattern)
    {
        ss << lookupTable[v] << "|";
    }

    return ss.str();
}

void
NrPhy::SetPowerAllocationType(enum NrSpectrumValueHelper::PowerAllocationType powerAllocationType)
{
    m_powerAllocationType = powerAllocationType;
}

enum NrSpectrumValueHelper::PowerAllocationType
NrPhy::GetPowerAllocationType() const
{
    return m_powerAllocationType;
}

void
NrPhy::EnqueueCtrlMessage(const Ptr<NrControlMessage>& m)
{
    NS_LOG_FUNCTION(this);

    m_controlMessageQueue.at(m_controlMessageQueue.size() - 1).push_back(m);
}

void
NrPhy::EnqueueCtrlMsgNow(const Ptr<NrControlMessage>& msg)
{
    NS_LOG_FUNCTION(this);

    m_controlMessageQueue.at(0).push_back(msg);
}

void
NrPhy::EnqueueCtrlMsgNow(const std::list<Ptr<NrControlMessage>>& listOfMsgs)
{
    for (const auto& msg : listOfMsgs)
    {
        m_controlMessageQueue.at(0).push_back(msg);
    }
}

void
NrPhy::EncodeCtrlMsg(const Ptr<NrControlMessage>& msg)
{
    NS_LOG_FUNCTION(this);
    m_ctrlMsgs.push_back(msg);
}

bool
NrPhy::HasDlSlot() const
{
    return NrPhy::HasDlSlot(m_tddPattern);
}

bool
NrPhy::HasUlSlot() const
{
    return NrPhy::HasUlSlot(m_tddPattern);
}

bool
NrPhy::HasDlSlot(const std::vector<LteNrTddSlotType>& pattern)
{
    for (const auto& v : pattern)
    {
        if (v == LteNrTddSlotType::F || v == LteNrTddSlotType::DL || v == LteNrTddSlotType::S)
        {
            return true;
        }
    }
    return false;
}

bool
NrPhy::HasUlSlot(const std::vector<LteNrTddSlotType>& pattern)
{
    for (const auto& v : pattern)
    {
        if (v == LteNrTddSlotType::F || v == LteNrTddSlotType::UL || v == LteNrTddSlotType::S)
        {
            return true;
        }
    }
    return false;
}

uint32_t
NrPhy::GetRbNum() const
{
    return m_rbNum;
}

uint32_t
NrPhy::GetChannelBandwidth() const
{
    // m_channelBandwidth is in kHz * 100
    return m_channelBandwidth * 1000 * 100;
}

uint32_t
NrPhy::GetSubcarrierSpacing() const
{
    return m_subcarrierSpacing;
}

void
NrPhy::DoUpdateRbNum()
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(m_channelBandwidth == 0, "Channel bandwidth not set");

    double realBw = GetChannelBandwidth() * (1 - m_rbOh);
    uint32_t rbWidth = m_subcarrierSpacing * NrSpectrumValueHelper::SUBCARRIERS_PER_RB;

    NS_ABORT_MSG_IF(
        rbWidth > realBw,
        "Bandwidth and numerology not correctly set. Bandwidth after reduction of overhead is :"
            << realBw << ", while RB width is: " << rbWidth);

    m_rbNum = static_cast<uint32_t>(realBw / rbWidth);
    NS_ASSERT(GetRbNum() > 0);

    NS_LOG_INFO("Updated RbNum to " << GetRbNum());

    NS_ASSERT(m_spectrumPhy);

    // Update the noisePowerSpectralDensity, as it depends on m_rbNum
    m_spectrumPhy->SetNoisePowerSpectralDensity(GetNoisePowerSpectralDensity());

    // once we have set noise power spectral density which will
    // initialize SpectrumModel of our SpectrumPhy, we can
    // call AddRx function of the SpectrumChannel
    if (m_spectrumPhy->GetSpectrumChannel())
    {
        m_spectrumPhy->GetSpectrumChannel()->AddRx(m_spectrumPhy);
    }
    else
    {
        NS_LOG_WARN("Working without channel (i.e., under test)");
    }
    NS_LOG_DEBUG("Noise Power Spectral Density updated");
}

bool
NrPhy::IsTdd(const std::vector<LteNrTddSlotType>& pattern)
{
    bool anUl = false;
    bool aDl = false;

    for (const auto& v : pattern)
    {
        // An F slot: we are TDD
        if (v == LteNrTddSlotType::F)
        {
            return true;
        }

        if (v == LteNrTddSlotType::UL)
        {
            anUl = true;
        }
        else if (v == LteNrTddSlotType::DL)
        {
            aDl = true;
        }
    }

    return !(anUl ^ aDl);
}

void
NrPhy::InitializeMessageList()
{
    NS_LOG_FUNCTION(this);
    m_controlMessageQueue.clear();

    for (unsigned i = 0; i <= GetL1L2CtrlLatency(); i++)
    {
        m_controlMessageQueue.emplace_back();
    }
}

std::list<Ptr<NrControlMessage>>
NrPhy::PopCurrentSlotCtrlMsgs()
{
    NS_LOG_FUNCTION(this);
    if (m_controlMessageQueue.empty())
    {
        std::list<Ptr<NrControlMessage>> emptylist;
        return (emptylist);
    }

    if (!m_controlMessageQueue.at(0).empty())
    {
        std::list<Ptr<NrControlMessage>> ret = m_controlMessageQueue.front();
        m_controlMessageQueue.erase(m_controlMessageQueue.begin());
        std::list<Ptr<NrControlMessage>> newlist;
        m_controlMessageQueue.push_back(newlist);
        return (ret);
    }
    else
    {
        m_controlMessageQueue.erase(m_controlMessageQueue.begin());
        std::list<Ptr<NrControlMessage>> newlist;
        m_controlMessageQueue.push_back(newlist);
        std::list<Ptr<NrControlMessage>> emptylist;
        return (emptylist);
    }
}

void
NrPhy::InstallSpectrumPhy(const Ptr<NrSpectrumPhy>& spectrumPhy)
{
    NS_LOG_FUNCTION(this);
    m_spectrumPhy = spectrumPhy;
}

void
NrPhy::SetBwpId(uint16_t bwpId)
{
    m_bwpId = bwpId;
}

uint16_t
NrPhy::GetBwpId() const
{
    return m_bwpId;
}

uint16_t
NrPhy::GetCellId() const
{
    return m_cellId;
}

uint32_t
NrPhy::GetL1L2CtrlLatency() const
{
    return 2;
}

Ptr<NrSpectrumPhy>
NrPhy::GetSpectrumPhy() const
{
    return m_spectrumPhy;
}

NrPhySapProvider*
NrPhy::GetPhySapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_phySapProvider;
}

void
NrPhy::PushBackSlotAllocInfo(const SlotAllocInfo& slotAllocInfo)
{
    NS_LOG_FUNCTION(this);

    NS_LOG_DEBUG("setting info for slot " << slotAllocInfo.m_sfnSf);

    // That's not so complex, as the list would typically be of 2 or 3 elements.
    bool updated = false;
    for (auto& alloc : m_slotAllocInfo)
    {
        if (alloc.m_sfnSf == slotAllocInfo.m_sfnSf)
        {
            NS_LOG_DEBUG("Merging inside existing allocation");
            alloc.Merge(slotAllocInfo);
            updated = true;
            break;
        }
    }
    if (!updated)
    {
        m_slotAllocInfo.push_back(slotAllocInfo);
        m_slotAllocInfo.sort();
        NS_LOG_DEBUG("Pushing allocation at the end of the list");
    }

    std::stringstream output;

    for (const auto& alloc : m_slotAllocInfo)
    {
        output << alloc;
    }
    NS_LOG_DEBUG(output.str());
}

void
NrPhy::PushFrontSlotAllocInfo(const SfnSf& newSfnSf, const SlotAllocInfo& slotAllocInfo)
{
    NS_LOG_FUNCTION(this);

    m_slotAllocInfo.push_front(slotAllocInfo);
    SfnSf currentSfn = newSfnSf;
    std::unordered_map<uint64_t, Ptr<PacketBurst>>
        newBursts;                                 // map between new sfn and the packet burst
    std::unordered_map<uint64_t, uint64_t> sfnMap; // map between new and old sfn, for debugging

    // all the slot allocations  (and their packet burst) have to be "adjusted":
    // directly modify the sfn for the allocation, and temporarily store the
    // burst (along with the new sfn) into newBursts.
    for (auto it = m_slotAllocInfo.begin(); it != m_slotAllocInfo.end(); ++it)
    {
        auto slotSfn = it->m_sfnSf;
        for (const auto& alloc : it->m_varTtiAllocInfo)
        {
            if (alloc.m_dci->m_type == DciInfoElementTdma::DATA)
            {
                Ptr<PacketBurst> pburst =
                    GetPacketBurst(slotSfn, alloc.m_dci->m_symStart, alloc.m_dci->m_rnti);
                if (pburst && pburst->GetNPackets() > 0)
                {
                    auto newKey = currentSfn.GetEncodingWithSymStartRnti(alloc.m_dci->m_symStart,
                                                                         alloc.m_dci->m_rnti);
                    auto oldKey = it->m_sfnSf.GetEncodingWithSymStartRnti(alloc.m_dci->m_symStart,
                                                                          alloc.m_dci->m_rnti);
                    newBursts.insert({newKey, pburst});
                    sfnMap.insert({newKey, oldKey});
                }
                else
                {
                    NS_LOG_INFO("No packet burst found for " << slotSfn);
                }
            }
        }

        NS_LOG_INFO("Set slot allocation for " << it->m_sfnSf << " to " << currentSfn);
        it->m_sfnSf = currentSfn;
        currentSfn.Add(1);
    }

    for (const auto& burstPair : newBursts)
    {
        SfnSf old;
        SfnSf latest;
        ns3::SfnSf::Decode(sfnMap.at(burstPair.first));
        ns3::SfnSf::Decode(burstPair.first);
        m_packetBurstMap.insert(std::make_pair(burstPair.first, burstPair.second));
        NS_LOG_INFO("PacketBurst with " << burstPair.second->GetNPackets() << "packets for SFN "
                                        << old << " now moved to SFN " << latest);
    }
}

bool
NrPhy::SlotAllocInfoExists(const SfnSf& retVal) const
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(retVal.GetNumerology() == GetNumerology());
    for (const auto& alloc : m_slotAllocInfo)
    {
        if (alloc.m_sfnSf == retVal)
        {
            return true;
        }
    }
    return false;
}

SlotAllocInfo
NrPhy::RetrieveSlotAllocInfo()
{
    NS_LOG_FUNCTION(this);
    SlotAllocInfo ret = *m_slotAllocInfo.begin();
    m_slotAllocInfo.erase(m_slotAllocInfo.begin());
    return ret;
}

SlotAllocInfo
NrPhy::RetrieveSlotAllocInfo(const SfnSf& sfnsf)
{
    NS_LOG_FUNCTION(" slot " << sfnsf);
    NS_ASSERT(sfnsf.GetNumerology() == GetNumerology());

    for (auto allocIt = m_slotAllocInfo.begin(); allocIt != m_slotAllocInfo.end(); ++allocIt)
    {
        if (allocIt->m_sfnSf == sfnsf)
        {
            SlotAllocInfo ret = *allocIt;
            m_slotAllocInfo.erase(allocIt);
            return ret;
        }
    }

    NS_FATAL_ERROR("Didn't found the slot");
    return SlotAllocInfo(sfnsf);
}

SlotAllocInfo&
NrPhy::PeekSlotAllocInfo(const SfnSf& sfnsf)
{
    NS_LOG_FUNCTION(this);
    NS_ASSERT(sfnsf.GetNumerology() == GetNumerology());
    for (auto& alloc : m_slotAllocInfo)
    {
        if (alloc.m_sfnSf == sfnsf)
        {
            return alloc;
        }
    }

    NS_FATAL_ERROR("Didn't found the slot");
}

size_t
NrPhy::SlotAllocInfoSize() const
{
    NS_LOG_FUNCTION(this);
    return m_slotAllocInfo.size();
}

bool
NrPhy::IsCtrlMsgListEmpty() const
{
    NS_LOG_FUNCTION(this);
    return m_controlMessageQueue.empty() || m_controlMessageQueue.at(0).empty();
}

Ptr<const SpectrumModel>
NrPhy::GetSpectrumModel()
{
    NS_LOG_FUNCTION(this);
    NS_ABORT_MSG_IF(GetSubcarrierSpacing() < 0.0, "Set a valid numerology");
    NS_ABORT_MSG_IF(m_channelBandwidth == 0, "Channel bandwidth not set.");
    return NrSpectrumValueHelper::GetSpectrumModel(GetRbNum(),
                                                   GetCentralFrequency(),
                                                   GetSubcarrierSpacing());
}

Time
NrPhy::GetSymbolPeriod() const
{
    // NS_LOG_FUNCTION(this);
    return m_symbolPeriod;
}

void
NrPhy::SetNoiseFigure(double d)
{
    m_noiseFigure = d;
    // as we don't know the order in which will be configured the parameters
    if (m_spectrumPhy && GetRbNum())
    {
        m_spectrumPhy->SetNoisePowerSpectralDensity(GetNoisePowerSpectralDensity());
    }
}

double
NrPhy::GetNoiseFigure() const
{
    return m_noiseFigure;
}

void
NrPhy::SetTbDecodeLatency(const Time& us)
{
    m_tbDecodeLatencyUs = us;
}

Time
NrPhy::GetTbDecodeLatency() const
{
    return m_tbDecodeLatencyUs;
}

// NR Sidelink

NrSlUePhySapProvider*
NrPhy::GetNrSlUePhySapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_nrSlUePhySapProvider;
}

Time
NrPhy::DoGetSlotPeriod() const
{
    return GetSlotPeriod();
}

uint32_t
NrPhy::DoGetBwInRbs() const
{
    return GetRbNum();
}

void
NrPhy::DoSendPscchMacPdu(Ptr<Packet> p)
{
    SetPscchMacPdu(p);
}

void
NrPhy::DoSendPscchMacPdu(Ptr<Packet> p, const SfnSf& sfnsf)
{
    SetPscchMacPdu(p, sfnsf);
}

void
NrPhy::DoSendPsschMacPdu(Ptr<Packet> p, uint32_t dstL2Id)
{
    SetPsschMacPdu(p, dstL2Id);
}

void
NrPhy::DoSendPsschMacPdu(Ptr<Packet> p, uint32_t dstL2Id, const SfnSf& sfnsf)
{
    SetPsschMacPdu(p, dstL2Id, sfnsf);
}

void
NrPhy::SetPscchMacPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);
    // Since we must send one SCI msg at a given time, initially the queue size must
    // be 1 with an empty packet burst
    NS_ASSERT_MSG(m_nrSlPscchPacketBurstQueue.size() == 1 &&
                      m_nrSlPscchPacketBurstQueue.at(0)->GetNPackets() == 0,
                  "Error in Pscch queue size and packet burst size (should be 1 and 0): "
                      << m_nrSlPscchPacketBurstQueue.size() << " "
                      << m_nrSlPscchPacketBurstQueue.at(0)->GetNPackets());
    m_nrSlPscchPacketBurstQueue.at(0)->AddPacket(p);
}

void
NrPhy::SetPscchMacPdu (Ptr<Packet> p, const SfnSf &sfn)
{
  NS_LOG_FUNCTION (m_mapNrSlPscchPacketBurstQueue.size () << sfn);
  if (m_mapNrSlPscchPacketBurstQueue.empty()){
    m_mapNrSlPscchPacketBurstQueue.push_back(std::map<SfnSf, Ptr<PacketBurst>>());
  }
  
  // store the data to the slPscchPAcket Burst queu
  std::map<SfnSf, Ptr<PacketBurst>>::iterator mapIt = m_mapNrSlPscchPacketBurstQueue.at(0).find(sfn);
  if (mapIt == m_mapNrSlPscchPacketBurstQueue.at (0).end()){
    Ptr<PacketBurst>pBurst = CreateObject <PacketBurst> ();
    pBurst->AddPacket(p);
    m_mapNrSlPscchPacketBurstQueue.at (0).insert(std::pair<SfnSf, Ptr<PacketBurst>>(sfn, pBurst));
  }else{
    mapIt->second->AddPacket(p);
  }
}

void
NrPhy::SetPsschMacPdu(Ptr<Packet> p, uint32_t dstL2Id)
{
    NS_LOG_FUNCTION(this << dstL2Id);
    // The packets in this packet burst would be equal to the number of LCs
    // multiplexed together plus one SCI format 2 packet
    auto it = m_nrSlPsschPacketBurstQueue.find(dstL2Id);
    if (it == m_nrSlPsschPacketBurstQueue.end())
    {
        Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
        pb->AddPacket(p);
        m_nrSlPsschPacketBurstQueue.insert({dstL2Id, pb});
    }
    else
    {
        it->second->AddPacket(p);
    }
}

void
NrPhy::SetPsschMacPdu(Ptr<Packet> p, uint32_t dstL2Id, const SfnSf& sfnsf)
{
    NS_LOG_FUNCTION(dstL2Id << m_nrSlPsschPacketBurstQueuePerSlot.size()<< sfnsf << p->GetSize());
    // The packets in this packet burst would be equal to the number of LCs
    // multiplexed together plus one SCI format 2 packet
    Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
    // Ptr<Packet> packet = p->Copy();
    auto it = m_nrSlPsschPacketBurstQueuePerSlot.find(sfnsf);
    if (it == m_nrSlPsschPacketBurstQueuePerSlot.end())
    {
        std::map<uint32_t, Ptr<PacketBurst>> new_map;
        pb->AddPacket(p);
        new_map.insert(std::pair<uint32_t, Ptr<PacketBurst>>(dstL2Id, pb));
        m_nrSlPsschPacketBurstQueuePerSlot.insert(std::pair<SfnSf, std::map<uint32_t, Ptr<PacketBurst>>>(sfnsf, new_map));
    }
    else
    {
        auto dstMapIt = it->second.find(dstL2Id);
        if(dstMapIt!=it->second.end()){
            // exist, so we add the data 
            dstMapIt->second->AddPacket(p);
        }else{
            pb->AddPacket(p);
            // the entry in the map does not exist so we add a new pair
            it->second.insert(std::pair<uint32_t, Ptr<PacketBurst>>(dstL2Id, pb));
        }
        
    }
}


Ptr<PacketBurst>
NrPhy::PopPscchPacketBurst()
{
    NS_LOG_FUNCTION(this);
    if (m_nrSlPscchPacketBurstQueue.at(0)->GetSize() > 0)
    {
        Ptr<PacketBurst> ret = m_nrSlPscchPacketBurstQueue.at(0)->Copy();
        m_nrSlPscchPacketBurstQueue.erase(m_nrSlPscchPacketBurstQueue.begin());
        m_nrSlPscchPacketBurstQueue.push_back(CreateObject<PacketBurst>());
        return (ret);
    }
    else
    {
        m_nrSlPscchPacketBurstQueue.erase(m_nrSlPscchPacketBurstQueue.begin());
        m_nrSlPscchPacketBurstQueue.push_back(CreateObject<PacketBurst>());
        return (nullptr);
    }
}

Ptr<PacketBurst>
NrPhy::PopPscchPacketBurst (const SfnSf &sfn)
{
  NS_LOG_FUNCTION (sfn);
  NS_ASSERT (m_mapNrSlPscchPacketBurstQueue.size () == 1);
  std::map<SfnSf, Ptr<PacketBurst>>::iterator mapIt = m_mapNrSlPscchPacketBurstQueue.at(0).find(sfn);
  if (mapIt != m_mapNrSlPscchPacketBurstQueue.at (0).end()){
    NS_LOG_DEBUG("Number of packets " << mapIt->second->GetNPackets());
    if(mapIt->second->GetSize () > 0){
      Ptr<PacketBurst> ret = mapIt->second->Copy ();
      // remove entry from the map after copying
      m_mapNrSlPscchPacketBurstQueue.at(0).erase (mapIt);
      return (ret);
    }else{
      return (0);
    }
  }else{
    return (0);
  }
}

Ptr<PacketBurst>
NrPhy::PopPsschPacketBurst()
{
    NS_LOG_FUNCTION(this);
    auto it = m_nrSlPsschPacketBurstQueue.begin();
    if (it != m_nrSlPsschPacketBurstQueue.end())
    {
        if (it->second->GetSize() > 0)
        {
            Ptr<PacketBurst> ret = it->second->Copy();
            m_nrSlPsschPacketBurstQueue.erase(it);
            return (ret);
        }
    }
    return (0);
}

Ptr<PacketBurst>
NrPhy::PopPsschPacketBurst(const SfnSf& sfn)
{
    NS_LOG_FUNCTION(m_nrSlPsschPacketBurstQueuePerSlot.size() << sfn);
    auto it = m_nrSlPsschPacketBurstQueuePerSlot.begin();
    Ptr<PacketBurst> ret = CreateObject<PacketBurst>();
    if (it != m_nrSlPsschPacketBurstQueuePerSlot.end())
    {
        for(auto dstMapIt = it->second.begin(); 
            dstMapIt!=it->second.end(); ++dstMapIt){
            if (dstMapIt->second!=nullptr){
                if(dstMapIt->second->GetSize()>0){
                    for (auto p: dstMapIt->second->GetPackets()){
                        NS_LOG_DEBUG("packet " << p << " size " << p->GetSize());
                        if ((p)->GetSize()>0){
                            ret->AddPacket((p)); //
                            NS_LOG_DEBUG("Packet addded");
                        }
                    }
                }
            }
        }
        m_nrSlPsschPacketBurstQueuePerSlot.erase(it);
        NS_LOG_DEBUG("Check burst size");
        if (ret->GetSize() > 0)
        {
            return (ret);
        }else{
            return (0);
        }
    }
    return (0);
}

void
NrPhy::DoSetNrSlVarTtiAllocInfo(const SfnSf& sfn, const NrSlVarTtiAllocInfo& varTtiInfo)
{
    NS_LOG_FUNCTION(this);
    // only one allocInfo should exist, which would be the slot allocation
    // info for the current slot.
    if (m_nrSlAllocInfoQueue.empty())
    {
        // new allocation
        NrSlPhySlotAlloc alloc;
        alloc.sfn = sfn;
        // each Var TTI in slvarTtiInfoList list must differ in their symbol start
        bool insertStatus = alloc.slvarTtiInfoList.emplace(varTtiInfo).second;
        NS_ASSERT_MSG(insertStatus,
                      "Insertion failed. A Var TTI starting from symbol " << varTtiInfo.symStart
                                                                          << " already exists");
        m_nrSlAllocInfoQueue.emplace_front(alloc);
    }
    else
    {
        auto it = m_nrSlAllocInfoQueue.begin();
        NS_ASSERT_MSG(it->sfn == sfn,
                      "Allocation queue must contain the allocation info of the same slot");
        // each Var TTI in slvarTtiInfoList list must differ in their symbol start
        bool insertStatus [[maybe_unused]] = it->slvarTtiInfoList.emplace(varTtiInfo).second;
    }
}

// bool
// NrPhy::NrSlSlotAllocInfoExists(const SfnSf& sfn) const
// {
//     NS_LOG_FUNCTION(this << sfn);
//     if (!m_nrSlAllocInfoQueue.empty())
//     {
//         return true;
//     }
//     for (auto it = m_nrSlPsschPacketBurstQueue.begin(); it != m_nrSlPsschPacketBurstQueue.end();
//          it++)
//     {
//         if (it->second->GetNPackets() > 0)
//         {
//             return true;
//         }
//     }
//     return false;
// }

bool
NrPhy::NrSlSlotAllocInfoExists(const SfnSf& sfn) const
{
    NS_LOG_FUNCTION(sfn << m_nrSlPsschPacketBurstQueuePerSlot.size());
    if (!m_nrSlAllocInfoQueue.empty())
    {
        return true;
    }
    for (auto it = m_nrSlPsschPacketBurstQueuePerSlot.begin(); it != m_nrSlPsschPacketBurstQueuePerSlot.end();
         it++)
    {
        for(auto dstIt = it->second.begin(); dstIt!=it->second.end(); ++dstIt){
            if (dstIt->second->GetNPackets() > 0)
            {
                return true;
            }
        }
        
    }
    return false;
}

} // namespace ns3
