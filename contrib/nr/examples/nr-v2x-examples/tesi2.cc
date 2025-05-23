/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/**
 * \ingroup examples
 * \file tesi.cc
 * \brief An project simulating NR V2V SL scenario with 6 sectors, where in each serctor 
 * there is one RSU and 6 vehicles. The vehicles are moving in a straight line with a
 * constant speed of  5 Km/h. The RSU is placed at the center of the sector. The vehicles
 * are sending data to the RSU using NR V2V SL communication in FR2 band, and the RSU is 
 * sending controlling data to the vehicles. In the scenario there are also 24 DO and 12 
 * UGV, sending packets to the gNB using FR1 bands. The gNB is placed at the center of the
 * scenario. The scenario is simulated for 1 second. The goal of this project is to evaluate
 * the performance of the NR V2V SL communication in a dense urban scenario using KPIs
 * as PRR, PIR, and throughput, delay.
 * 
 *
 * \code{.unparsed}
$ ./ns3 run "tesi --help"
    \endcode
 *
 */

#include "v2x-kpi.h"
#include <cmath>
#include "ns3/flow-monitor-module.h"
#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include <random>
#include <iomanip>

#include "stats_func.h"

#include "ns3/config-store-module.h"
#include "ns3/config-store.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("tesi");

// SOLO SIDELINK multilink

 

int
main(int argc, char* argv[])
{
    // 1. PARAMETRI: solo 1 RSU e 1 CAV
    uint16_t numCAVsPerSector = 3; // 1 RSU + 1 CAV
    uint16_t numSector = 1;
    int distance = 100;
    double CAVsSpeed = 5.0 / 3.6; // m/s
    bool enableOneRxPerSector = true;
    bool logging = true;
    bool harqEnabled = true;
    bool enablemoregnb = false;
    Time delayBudget = MilliSeconds(0); // Use T2 configuration
    // Time delayBudget = MicroSeconds(1000); // Use T2 configuration

    // Traffic parameters 
    bool useIPv6 = false; // default IPV4
    uint32_t udpPacketSize4k = 1500;
    uint32_t udpPacketSizeControl = 200;

    double dataRatecavrsu = 120256.0; // 120.256 Mbps per second
    // double dataRatecavrsu = 1202.0; // 120.256 Mbps per second
    double dataRatersucav = 200.0; // 200 kbps
    double dataRatecavcav = 200.0; // 200 kbps

    // Simulation parameters.
    Time simTime = Seconds(20.0);
    // Sidelink bearers activation time
    Time slBearersActivationTime = Seconds(0.1);

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    double centralFrequencyBandSl = 26.5e9; // band n257  TDD //Here band is analogous to channel
    uint16_t bandwidthBandSl = 2000;         // Multiple of 100 KHz; 2000=200 MHz
    std::string errorModel = "ns3::NrEesmIrT1";
    std::string beamformingMethod = "ns3::DirectPathBeamforming";

    double txPower = 29.0;                    // dBm
    std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
    std::string slBitMap = "1|1|1|1|1|1|1|1|1|1|";
    uint16_t numerologyBwpSl = 3;
    uint16_t slSensingWindow = 100; // T0 in ms
    uint16_t slSelectionWindow = 1; // T2min
    uint16_t slSubchannelSize = 100;
    uint16_t slMaxNumPerReserve = 1;
    double slProbResourceKeep = 0.0;
    uint16_t slMaxTxTransNumPssch = 2;
    uint16_t reservationPeriod = 10; // in ms
    uint16_t t1 = 1;
    uint16_t t2 = 15;
    int slThresPsschRsrp = -128;
    uint16_t channelUpdatePeriod = 500; // ms
    uint8_t mcs = 28;

    // flags to generate gnuplot plotting scripts
    bool generateInitialPosGnuScript = true;
    bool generateGifGnuScript = false;

    bool enableChannelRandomness = true;
    bool enableSensing = true;
    // Where we will store the output files.
    // std::string outputDir = "/mnt/c/Users/filod/Desktop/tesi/Risultati/db/";
    std::string outputDir = "/home/fgjeci/Desktop/test/";
    std::string exampleName = "prova_onlysl2t2_32";

    CommandLine cmd(__FILE__);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("numCAVsPerSector", "Number of vehicles per numSector", numCAVsPerSector);
    cmd.AddValue("numSector", "Total Number of numSector", numSector);
    cmd.AddValue("CAVsSpeed", "Speed of the CAVs in m/sec", CAVsSpeed);
    cmd.AddValue("distance", "Distance of the sectors from the origins", distance);
    cmd.AddValue("enableOneRxPerSector",
                 "Per lane make one vehicle a transmitter. This option only works"
                 "with odd number of UEs per lane, which makes the middle vehicle"
                 "in each lane a transmitter",
                 enableOneRxPerSector);
    cmd.AddValue("useIPv6", "Use IPv6 instead of IPv4", useIPv6);
    cmd.AddValue("packetSizeBe",
                 "packet size in bytes to be used by 4k video survelience traffic",
                 udpPacketSize4k);
    cmd.AddValue("packetSizeControl",
                 "packet size in bytes to be used by control traffic",
                 udpPacketSizeControl);
    cmd.AddValue("dataRatecavrsu",
                 "The data rate in kilobits per second for cav->rsu traffic",
                 dataRatecavrsu);
    cmd.AddValue("dataRatersucav",
                 "The data rate in kilobits per second for rsu->cav traffic",
                 dataRatersucav);
    cmd.AddValue("dataRatecavcav",
                 "The data rate in kilobits per second for cav->cav traffic",
                 dataRatecavcav);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("slBearerActivationTime",
                 "Sidelik bearer activation time in seconds",
                 slBearersActivationTime);
    cmd.AddValue("centralFrequencyBandSl",
                 "The central frequency to be used for Sidelink band/channel",
                 centralFrequencyBandSl);
    cmd.AddValue("bandwidthBandSl",
                 "The system bandwidth to be used for Sidelink",
                 bandwidthBandSl);
    cmd.AddValue("txPower", "total tx power in dBm", txPower);
    cmd.AddValue("tddPattern", "The TDD pattern string", tddPattern);
    cmd.AddValue("slBitMap", "The Sidelink bitmap string", slBitMap);
    cmd.AddValue("numerologyBwpSl",
                 "The numerology to be used in Sidelink bandwidth part",
                 numerologyBwpSl);   
    cmd.AddValue("slSensingWindow", "The Sidelink sensing window length in ms", slSensingWindow);
    cmd.AddValue("slSelectionWindow",
                 "The parameter which decides the minimum Sidelink selection "
                 "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                 slSelectionWindow);
    cmd.AddValue("slSubchannelSize", "The Sidelink subchannel size in RBs", slSubchannelSize);
    cmd.AddValue("slMaxNumPerReserve",
                 "The parameter which indicates the maximum number of reserved "
                 "PSCCH/PSSCH resources that can be indicated by an SCI.",
                 slMaxNumPerReserve);
    cmd.AddValue("slProbResourceKeep",
                 "The parameter which indicates the probability with which the "
                 "UE keeps the current resource when the resource reselection"
                 "counter reaches zero.",
                 slProbResourceKeep);
    cmd.AddValue("slMaxTxTransNumPssch",
                 "The parameter which indicates the maximum transmission number "
                 "(including new transmission and retransmission) for PSSCH.",
                 slMaxTxTransNumPssch);
    cmd.AddValue("enableSensing",
                 "If true, it enables the sensing based resource selection for "
                 "SL, otherwise, no sensing is applied",
                 enableSensing);
    cmd.AddValue("t1",
                 "The start of the selection window in physical slots, "
                 "accounting for physical layer processing delay",
                 t1);
    cmd.AddValue("t2", "The end of the selection window in physical slots", t2);
    cmd.AddValue("slThresPsschRsrp",
                 "A threshold in dBm used for sensing based UE autonomous resource selection",
                 slThresPsschRsrp);
    cmd.AddValue("enableChannelRandomness",
                 "Enable shadowing and channel updates",
                 enableChannelRandomness);
    cmd.AddValue("channelUpdatePeriod", "The channel update period in ms", channelUpdatePeriod);
    cmd.AddValue("mcs", "The MCS to used for sidelink", mcs);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);
    cmd.AddValue("generateInitialPosGnuScript",
                 "generate gnuplot script to plot initial positions of the UEs",
                 generateInitialPosGnuScript);
    cmd.AddValue("generateGifGnuScript",
                 "generate gnuplot script to generate GIF to show UEs mobility",
                 generateGifGnuScript);
    cmd.AddValue("exampleName", "The name of the dbfile to save results", exampleName);
    


    // Parse the command line
    cmd.Parse(argc, argv);

    if (logging)
    {
        LogLevel logLevel =
            (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        LogComponentEnable("tesi", logLevel);
        // LogComponentEnable("LtePdcp", logLevel);
        // LogComponentEnable("NrSlHelper", logLevel);
        // LogComponentEnable("NrHelper", logLevel);
        // LogComponentEnable("NrSlUeRrc", logLevel);
        // LogComponentEnable("NrUeMac", logLevel);
        // LogComponentEnable("NrSlUeMac", logLevel);
        // LogComponentEnable("NrUePhy", logLevel);
        // LogComponentEnable("NrUeNetDevice", logLevel);
        // LogComponentEnable("NrNetDevice", logLevel);
        // LogComponentEnable("NrPhy", logLevel);
        // LogComponentEnable("NrSpectrumPhy", logLevel);
        // LogComponentEnable("NrGnbMac", logLevel);
        // LogComponentEnable("NrGnbNetDevice", logLevel);
        // LogComponentEnable("NrGnbPhy", logLevel);   
        // LogComponentEnable("PacketSink", logLevel);
        // LogComponentEnable("LteUeRrc", logLevel);
        // LogComponentEnable("UdpServer", logLevel);
        // LogComponentEnable("Ipv4L3Protocol", logLevel);
        // LogComponentEnable("UdpL4Protocol", logLevel);
        // LogComponentEnable("EpcUeNas", logLevel);
        // LogComponentEnable("LteEnbRrc", logLevel);
        LogComponentEnable("NrSlUeMacSchedulerFixedMcs", logLevel);
        // LogComponentEnable("LteRlcUm", logLevel);
        // LogComponentEnable("NrBearerStatsConnector", logLevel);
        
        
        // LogComponentEnable("LteUeComponentCarrierManager", logLevel);
        // LogComponentEnableAll(logLevel);
        // LogComponentEnable("NrSlBwpManagerUe", logLevel);
    }

        

    /*
     * Default values for the simulation.
     */
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::LteRlcUm::WindowSize", UintegerValue(512));
    Config::SetDefault("ns3::LteRlcUm::ReorderingTimer", TimeValue (MilliSeconds (10)));

    // Config::SetDefault("ns3::NrSlUeMac::ResourcePercentage", UintegerValue(50));
    
    
    /*
     * Create a NodeContainer for all the UEs
     */
    NodeContainer allSlUesContainer;

    /*
     * Assign mobility to the UEs.
     *  1. Set mobility model type.
     *  2. Assign position to the UEs
     *  3. Install mobility model
     */
    allSlUesContainer = InstallSLMobility(numSector, numCAVsPerSector, CAVsSpeed, distance);

    Packet::EnableChecking();
    Packet::EnablePrinting();
    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - EpcHelper, which will setup the core network
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetAttribute("TracesPath", StringValue(outputDir));
    nrHelper->SetEpcHelper(epcHelper);
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    
    /*
     * Spectrum division. We create one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * We will use the StreetCanyon channel modeling.
     */

    BandwidthPartInfoPtrVector allBwps;
    BandwidthPartInfoPtrVector allBwpsSL;
    BandwidthPartInfoPtrVector allBwpsFR1;

    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
     * creates a single BWP per CC
     */

    // By using the configuration created, it is time to make the operation bands
    // bandFR1.isSidelink=false;
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    // By using the configuration created, it is time to make the operation bands
    bandConfSl.isSidelink=true;
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);
    // bandSl.isSidelink=true;
    

    /*
     * The configured spectrum division is:
     * ------------Band1-----------------Band2--------------
     * ------------CC1-------------------CC1--------------
     * ------------BwpSl-----------------BandFR1------------
     */
    if (enableChannelRandomness)
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                           TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                    TimeValue(MilliSeconds(channelUpdatePeriod)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(true));
    }
    else
    {
        Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(0)));
        nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(100)));
        nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    }

    /*
     * Initialize channel and pathloss, plus other things inside bandSl. If needed,
     * the band configuration can be done manually, but we leave it for more
     * sophisticated examples. For the moment, this method will take care
     * of all the spectrum initialization needs.
     */ 
    nrHelper->InitializeOperationBand(&bandSl);
    allBwpsSL = CcBwpCreator::GetAllBwps({bandSl});
   
    /*
     * Antennas for all the UEs
     * Using quasi-omnidirectional transmission and reception, which is the default
     * configuration of the beams.
     *
     * Following attribute would be common for all the UEs
     */
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(8));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(8));
    Ptr<IsotropicAntennaModel> ueIsotropicAntenna = CreateObject<IsotropicAntennaModel>();
    ueIsotropicAntenna->SetAttribute("Gain", DoubleValue(11.0));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueIsotropicAntenna));
    
    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(TypeId::LookupByName(beamformingMethod)));
                                          // Core latency
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));
   
    nrHelper->SetUeMacTypeId(NrUeMac::GetTypeId());
    // nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(enableSensing));
    // nrHelper->SetUeMacAttribute("T1", UintegerValue(static_cast<uint8_t>(t1)));
    // nrHelper->SetUeMacAttribute("T2", UintegerValue(t2));
    // nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));
    // nrHelper->SetUeMacAttribute("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));
    
    // uint8_t bwpIdForGbrMcptt = 0;
    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    // gNb routing between bearer type and bandwidth part
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(0));
    
    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    // std::vector<ObjectFactory> m_ueSlMacFactory;
    // for (uint8_t _i = 0; _i< numCcPerBand; ++_i){
    //     ObjectFactory ueMacFactory;
    //     ueMacFactory.SetTypeId(NrSlUeMac::GetTypeId());
    //     ueMacFactory.Set("EnableSensing", BooleanValue(enableSensing));
    //     ueMacFactory.Set("T1", UintegerValue(static_cast<uint8_t>(t1)));
    //     ueMacFactory.Set("T2", UintegerValue(t2));
    //     ueMacFactory.Set("ActivePoolId", UintegerValue(0));
    //     ueMacFactory.Set("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));
    //     m_ueSlMacFactory.push_back(ueMacFactory);
    // }   
    ObjectFactory m_ueSlMacFactory;
    m_ueSlMacFactory.SetTypeId(NrSlUeMac::GetTypeId());
    m_ueSlMacFactory.Set("EnableSensing", BooleanValue(enableSensing));
    m_ueSlMacFactory.Set("T1", UintegerValue(static_cast<uint8_t>(t1)));
    m_ueSlMacFactory.Set("T2", UintegerValue(t2));
    m_ueSlMacFactory.Set("ActivePoolId", UintegerValue(0));
    m_ueSlMacFactory.Set("SlThresPsschRsrp", IntegerValue(slThresPsschRsrp));

    ObjectFactory m_ueMacFactory;
    m_ueMacFactory.SetTypeId(NrUeMac::GetTypeId());

    NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice(allSlUesContainer, allBwpsSL, m_ueMacFactory, m_ueSlMacFactory);

    for (auto it = allSlUesNetDeviceContainer.Begin(); it != allSlUesNetDeviceContainer.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    /*
     * Configure Sidelink. We create the following helpers needed for the
     * NR Sidelink, i.e., V2X simulation:
     * - NrSlHelper, which will configure the UEs protocol stack to be ready to
     *   perform Sidelink related procedures.
     * - EpcHelper, which takes care of triggering the call to EpcUeNas class
     *   to establish the NR Sidelink bearer(s). We note that, at this stage
     *   just communicate the pointer of already instantiated EpcHelper object,
     *   which is the same pointer communicated to the NrHelper above.
     */
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    // Put the pointers inside NrSlHelper
    nrSlHelper->SetEpcHelper(epcHelper);

    /*
     * Set the SL error model and AMC
     * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
     *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
     * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
     */
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    /*
     * Set the SL scheduler attributes
     * In this i use NrSlUeMacSchedulerSimple scheduler, which uses
     * a fixed MCS value
     */
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(mcs));

    /*
     * Very important method to configure UE protocol stack, i.e., it would
     * configure all the SAPs among the layers, setup callbacks, configure
     * error model, configure AMC, and configure ChunkProcessor in Interference
     * API.
     */
    nrSlHelper->PrepareUeForSidelink(allSlUesNetDeviceContainer, bwpIdContainer);

    /*
     * Start preparing for all the sub Structs/RRC Information Element (IEs)
     * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
     * hold all the pre-configuration related to Sidelink.
     */

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    /*
     * Above pool factory is created to help the users of the simulator to create
     * a pool with valid default configuration. Please have a look at the
     * constructor of NrSlCommResourcePoolFactory class.
     */
    std::vector<std::bitset<1>> slBitMapVector;
    GetSlBitmapFromString(slBitMap, slBitMapVector);
    NS_ABORT_MSG_IF(slBitMapVector.empty(), "GetSlBitmapFromString failed to generate SL bitmap");
    ptrFactory->SetSlTimeResources(slBitMapVector);
    ptrFactory->SetSlSensingWindow(slSensingWindow); // T0 in ms
    ptrFactory->SetSlSelectionWindow(slSelectionWindow);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(slSubchannelSize);
    ptrFactory->SetSlMaxNumPerReserve(slMaxNumPerReserve);
    std::list<uint16_t> resourceReservePeriodList = {0, reservationPeriod}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = tddPattern;

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    NS_ABORT_MSG_UNLESS(slProbResourceKeep <= 1.0,
                        "slProbResourceKeep value must be between 0 and 1");
    slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = static_cast<uint8_t>(slMaxTxTransNumPssch);
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr. This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(allSlUesNetDeviceContainer, slPreConfigNr);

    /****************************** End SL Configuration ***********************/

    /*
     * Fix the random streams
     */
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(allSlUesNetDeviceContainer, stream);
    stream += nrSlHelper->AssignStreams(allSlUesNetDeviceContainer, stream);
    
    /*
    
     * if enableOneRxPerSector is true:
     *
     * All the UEs can transmit and receive
     */
    NodeContainer CavUEs;
    NodeContainer RsuUes;
    NetDeviceContainer CavUEsNetDevice;
    NetDeviceContainer RsuUesNetDevice;

    for (uint16_t i = 0; i < numSector * numCAVsPerSector; i++)
        {
            // for each Sector one rsu and rest cav
            if (i % numCAVsPerSector == 0) {
                RsuUes.Add(allSlUesContainer.Get(i));
                RsuUesNetDevice.Add(allSlUesContainer.Get(i)->GetDevice(0));
            }else{  
                CavUEs.Add(allSlUesContainer.Get(i));
                CavUEsNetDevice.Add(allSlUesContainer.Get(i)->GetDevice(0));  
            }
        }

    /*
     * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
     * configured time.
     *
     * This example supports IPV4 and IPV6
     */

    InternetStackHelper internet;
    internet.Install(allSlUesContainer);
    stream += internet.AssignStreams(allSlUesContainer, stream);

    uint32_t dstL2Groupcast = 254;
    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    Ipv6Address groupAddress6("ff0e::1");   // use multicast address as destination
    //sl adresses
    Address remoteAddress;
    Address localAddress;
    Address remoteAddress2;
    Address localAddress2;
    Address remoteAddress3;
    Address localAddress3;

    //sl ports
    uint16_t port = 8000;
    uint16_t port2 = 8001;
    uint16_t port3 = 8002;

    Ptr<LteSlTft> tft;

    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Groupcast;
    slInfo.m_rri = MilliSeconds(reservationPeriod);
    slInfo.m_dynamic = false;
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;

    SidelinkInfo slInfo2;
    slInfo2.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo2.m_dstL2Id = dstL2Groupcast;
    slInfo2.m_rri = MilliSeconds(reservationPeriod);
    slInfo2.m_dynamic = false;
    slInfo2.m_pdb = delayBudget;
    slInfo2.m_harqEnabled = harqEnabled;
    
    SidelinkInfo slInfo3;
    slInfo3.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo3.m_dstL2Id = dstL2Groupcast;
    slInfo3.m_rri = MilliSeconds(reservationPeriod);
    slInfo3.m_dynamic = false;
    slInfo3.m_pdb = delayBudget;
    slInfo3.m_harqEnabled = harqEnabled;

    if (!useIPv6)
    {
        Ipv4InterfaceContainer ueSlIpIface;
        ueSlIpIface= epcHelper->AssignUeIpv4Address(allSlUesNetDeviceContainer);
        
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (uint32_t u = 0; u < allSlUesContainer.GetN(); ++u)
        {
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(allSlUesContainer.Get(u)->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }

        //sl prova
        remoteAddress = InetSocketAddress(groupAddress4, port);
        localAddress = InetSocketAddress(Ipv4Address::GetAny(), port);
        remoteAddress2 = InetSocketAddress(groupAddress4, port2);
        localAddress2 = InetSocketAddress(Ipv4Address::GetAny(), port2);
        remoteAddress3 = InetSocketAddress(groupAddress4, port3);
        localAddress3 = InetSocketAddress(Ipv4Address::GetAny(), port3);
        
        tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port, slInfo);
        // Set Sidelink bearers 
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);  
        tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, RsuUesNetDevice, tft);  
        tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, port3, slInfo3);
        // Set Sidelink bearers 
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);

        tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, groupAddress4, port2, slInfo2);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, RsuUesNetDevice, tft);
        
        tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, groupAddress4, port2, slInfo2);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);
        // 
        // tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, groupAddress4, port3, slInfo3);
        // // Set Sidelink bearers
        // nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);
        // 
        // tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, groupAddress4, port3, slInfo3);
        // // Set Sidelink bearers
        // nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);
    }
    else
    {
        Ipv6InterfaceContainer ueIpIface, ueSlIpIface;
        ueIpIface = epcHelper->AssignUeIpv6Address(allSlUesNetDeviceContainer);

        // set the default gateway for the UE
        Ipv6StaticRoutingHelper ipv6RoutingHelper;
        for (uint32_t u = 0; u < allSlUesContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = allSlUesContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv6StaticRouting> ueStaticRouting =
                ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);
        }
        remoteAddress = Inet6SocketAddress(groupAddress6, port);
        localAddress = Inet6SocketAddress(Ipv6Address::GetAny(), port);

        tft = Create<LteSlTft>(LteSlTft::Direction::TRANSMIT, groupAddress4, slInfo);
        // Set Sidelink bearers for transmitting UEs
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, CavUEsNetDevice, tft);

        tft = Create<LteSlTft>(LteSlTft::Direction::RECEIVE, groupAddress4, slInfo);
        nrSlHelper->ActivateNrSlBearer(slBearersActivationTime, RsuUesNetDevice, tft);
    }
   
    /*
     * Configure the applications:
     * Client app: OnOff application configure to generate CBR traffic
     * Server app: PacketSink application.
     */
    // Random variable to randomize a bit start times of the client applications
    // to avoid simulation artifacts of all the TX UEs transmitting at the same time.
    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetStream(stream);
    startTimeSeconds->SetAttribute("Min", DoubleValue(0));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.10));

    // Set Application in the UEs
    OnOffHelper sidelinkClient("ns3::UdpSocketFactory", remoteAddress);
    sidelinkClient.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string dataRatecavrsuString = std::to_string(dataRatecavrsu) + "kb/s";
    NS_LOG_INFO( "Data rate " << DataRate(dataRatecavrsuString) );
    sidelinkClient.SetConstantRate(DataRate(dataRatecavrsuString), udpPacketSize4k);

    OnOffHelper sidelinkClient2("ns3::UdpSocketFactory", remoteAddress2);
    sidelinkClient2.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string dataRatersucavString = std::to_string(dataRatersucav) + "kb/s";
    NS_LOG_INFO( "Data rate " << DataRate(dataRatersucavString) );
    sidelinkClient2.SetConstantRate(DataRate(dataRatersucavString), udpPacketSizeControl);

    OnOffHelper sidelinkClient3("ns3::UdpSocketFactory", remoteAddress3);
    sidelinkClient3.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string dataRatecavcavString = std::to_string(dataRatecavcav) + "kb/s";
    NS_LOG_INFO( "Data rate " << DataRate(dataRatecavcavString) );
    sidelinkClient3.SetConstantRate(DataRate(dataRatecavcavString), udpPacketSizeControl);
    
    

    ApplicationContainer clientApps;
    ApplicationContainer clientApps2;
    ApplicationContainer clientApps3;

    double realAppStart = 0.0;
    double realAppStopTime = 0.0;
    double txAppDuration = 0.0;
    double txAppDurationFin = 0.0;
    
    std::ostringstream path;
    int j=0;
 
    for (uint32_t i = 0; i < CavUEs.GetN(); i++)
    {
        clientApps.Add(sidelinkClient.Install(CavUEs.Get(i)));
        double jitter = startTimeSeconds->GetValue();
        Time appStart = slBearersActivationTime + Seconds(jitter);
        clientApps.Get(i)->SetStartTime(appStart);

        // trace added start
        // Ipv4Address localAddrs =  clientApps.Get (i)->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
        // NS_LOG_INFO( "Mbs Tx address: " << localAddrs);
        // clientApps.Get (i)->TraceConnect ("TxWithSeqTsSize", "tx", MakeBoundCallback (&UePacketTraceDb, &pktStats, clientApps.Get (i)->GetNode (), localAddrs));
        // traces added end

        // onoff application will send the first packet at :
        // slBearersActivationTime + random jitter + ((Pkt size in bits) / (Data rate in bits per
        // sec))
        realAppStart = slBearersActivationTime.GetSeconds() + jitter +
                       ((double)udpPacketSize4k * 8.0 / (DataRate(dataRatecavrsuString).GetBitRate()));
        realAppStopTime = realAppStart + simTime.GetSeconds();
        clientApps.Get(i)->SetStopTime(Seconds(realAppStopTime));
        txAppDuration = realAppStopTime - realAppStart;

        // Output app start, stop and duration
        NS_LOG_INFO( "Tx App " << j + 1 << " start time " << std::fixed << std::setprecision(5)
                  << realAppStart << " sec" );
        NS_LOG_INFO( "Tx App " << j + 1 << " stop time " << realAppStopTime << " sec" );
        NS_LOG_INFO( "Tx App duration " << std::defaultfloat << txAppDuration << " sec"
                  );

        if(txAppDuration > txAppDurationFin){
            txAppDurationFin = txAppDuration;
        }

        j++;

        //app3 for cav cav link
        clientApps3.Add(sidelinkClient3.Install(CavUEs.Get(i)));
        double jitter1 = startTimeSeconds->GetValue();
        Time appStart1 = slBearersActivationTime + Seconds(jitter1);
        clientApps3.Get(i)->SetStartTime(appStart1);

        // trace added start
        // Ipv4Address localAddrs3 =  clientApps3.Get (i)->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
        // NS_LOG_INFO( "Mbs Tx address: " << localAddrs3);
        // clientApps3.Get (i)->TraceConnect ("TxWithSeqTsSize", "tx", MakeBoundCallback (&EfStatsHelper::UePacketTraceDb, &pktStats, clientApps3.Get (i)->GetNode (), localAddrs3));
        // traces added end

        // onoff application will send the first packet at :
        // slBearersActivationTime + random jitter + ((Pkt size in bits) / (Data rate in bits per
        // sec))
        realAppStart = slBearersActivationTime.GetSeconds() + jitter1 +
                       ((double)udpPacketSizeControl * 8.0 / (DataRate(dataRatecavcavString).GetBitRate()));
        realAppStopTime = realAppStart + simTime.GetSeconds();
        clientApps3.Get(i)->SetStopTime(Seconds(realAppStopTime));
        txAppDuration = realAppStopTime - realAppStart;

        // Output app start, stop and duration
        NS_LOG_INFO( "Tx App " << j + 1 << " start time " << std::fixed << std::setprecision(5)
                  << realAppStart << " sec" );
        NS_LOG_INFO( "Tx App " << j + 1 << " stop time " << realAppStopTime << " sec" );
        NS_LOG_INFO( "Tx App duration " << std::defaultfloat << txAppDuration << " sec"
                  );
        
        if(txAppDuration > txAppDurationFin){
            txAppDurationFin = txAppDuration;
        }
        j++;
    }
    //app for rsu  link
    for (uint32_t i = 0; i < RsuUes.GetN(); i++)
    {
        clientApps2.Add(sidelinkClient2.Install(RsuUes.Get(i)));
        double jitter2 = startTimeSeconds->GetValue();
        Time appStart = slBearersActivationTime + Seconds(jitter2);
        clientApps2.Get(i)->SetStartTime(appStart);

        // traces added start
        // Ipv4Address localAddrs =  clientApps2.Get (i)->GetNode ()->GetObject<Ipv4L3Protocol> ()->GetAddress (1,0).GetLocal ();
        // NS_LOG_INFO( "Mbs Tx address: " << localAddrs);
        // clientApps2.Get (i)->TraceConnect ("TxWithSeqTsSize", "tx", MakeBoundCallback (&UePacketTraceDb, &pktStats, clientApps2.Get (i)->GetNode (), localAddrs));
        // traces added end

        // onoff application will send the first packet at :
        // slBearersActivationTime + random jitter + ((Pkt size in bits) / (Data rate in bits per
        // sec))
        realAppStart = slBearersActivationTime.GetSeconds() + jitter2 +
                       ((double)udpPacketSizeControl * 8.0 / (DataRate(dataRatecavcavString).GetBitRate()));
        realAppStopTime = realAppStart + simTime.GetSeconds();
        clientApps2.Get(i)->SetStopTime(Seconds(realAppStopTime));
        txAppDuration = realAppStopTime - realAppStart;

        // Output app start, stop and duration
        NS_LOG_INFO( "Tx App " << j + 1 << " start time " << std::fixed << std::setprecision(5)
                  << realAppStart << " sec" );
        NS_LOG_INFO( "Tx App " << j + 1 << " stop time " << realAppStopTime << " sec" );
        NS_LOG_INFO( "Tx App duration " << std::defaultfloat << txAppDuration << " sec"
                  );

        if(txAppDuration > txAppDurationFin){
            txAppDurationFin = txAppDuration;
        }
        j++;
    }
    ApplicationContainer serverApps;
    PacketSinkHelper sidelinkSink("ns3::UdpSocketFactory", localAddress);
    sidelinkSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    ApplicationContainer serverApps2;
    PacketSinkHelper sidelinkSink2("ns3::UdpSocketFactory", localAddress2);
    sidelinkSink2.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    ApplicationContainer serverApps3;
    PacketSinkHelper sidelinkSink3("ns3::UdpSocketFactory", localAddress3);
    sidelinkSink3.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    for (uint32_t i = 0; i < RsuUes.GetN(); i++)
    {
        serverApps.Add(sidelinkSink.Install(RsuUes.Get(i)));
        serverApps.Start(Seconds(0.0));
    }
    for (uint32_t i = 0; i < CavUEs.GetN(); i++)
    {
        serverApps2.Add(sidelinkSink2.Install(CavUEs.Get(i)));
        serverApps2.Start(Seconds(0.0));
        serverApps3.Add(sidelinkSink3.Install(CavUEs.Get(i)));
        serverApps3.Start(Seconds(0.0));
        
    }

    
    /*
     * Hook the traces, for trace data to be stored in a database
     */
    //std::string exampleName = "tesi250m";
    // Datebase setup
    SQLiteOutput db(outputDir + exampleName + ".db");
    uint32_t writeCacheSize = 1000; // 2MB

    UeMacPscchTxOutputStats pscchStats;
    pscchStats.SetDb(&db, "pscchTxUeMac", writeCacheSize);
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPscchScheduling",
                                  MakeBoundCallback(&NotifySlPscchScheduling, &pscchStats));

    UeMacPsschTxOutputStats psschStats;
    psschStats.SetDb(&db, "psschTxUeMac", writeCacheSize);
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPsschScheduling",
                                  MakeBoundCallback(&NotifySlPsschScheduling, &psschStats));

    UePhyPscchRxOutputStats pscchPhyStats;
    pscchPhyStats.SetDb(&db, "pscchRxUePhy", writeCacheSize);
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPscchTraceUe",
        MakeBoundCallback(&NotifySlPscchRx, &pscchPhyStats));

    UePhyPsschRxOutputStats psschPhyStats;
    psschPhyStats.SetDb(&db, "psschRxUePhy", writeCacheSize);
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPsschTraceUe",
        MakeBoundCallback(&NotifySlPsschRx, &psschPhyStats));

    UeRlcRxOutputStats ueRlcRxStats;
    ueRlcRxStats.SetDb(&db, "rlcRx", writeCacheSize);
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/RxRlcPduWithTxRnti",
                                  MakeBoundCallback(&NotifySlRlcPduRx, &ueRlcRxStats));

    

    

    //SlCtrlMsgsStats ctrlMsgsStats;
    //ctrlMsgsStats.SetDb(&db, "ctrlMsgsStats", writeCacheSize);
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/*/NrGnbPhy/GnbPhyRxedCtrlMsgsTrace",
    //    MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "PHY", "Gnb Rxed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/*/NrGnbPhy/GnbPhyTxedCtrlMsgsTrace",
    //        MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "PHY", "Gnb Txed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/*/NrGnbMac/GnbMacRxedCtrlMsgsTrace",
    //        MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "MAC", "Gnb Rxed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrGnbNetDevice/BandwidthPartMap/*/NrGnbMac/GnbMacTxedCtrlMsgsTrace",
    //        MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "MAC", "Gnb Txed"));
    //
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUeMac/UeMacRxedCtrlMsgsTrace",
    //            MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "MAC", "Ue Rxed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUeMac/UeMacTxedCtrlMsgsTrace",
    //            MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "MAC", "Ue Txed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedCtrlMsgsTrace",
    //            MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "PHY", "Ue Rxed"));
    //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedCtrlMsgsTrace",
    //MakeBoundCallback(&ReportCtrlMessages, &ctrlMsgsStats, "PHY", "Ue Txed"));

    UeToUePktTxRxOutputStats pktStats;
    pktStats.SetDb(&db, "pktTxRx", writeCacheSize);

    if (!useIPv6)
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Tx address: " << localAddrs );
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }
        for (uint32_t ac = 0; ac < clientApps2.GetN(); ac++)
        {
            Ipv4Address localAddrs = clientApps2.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Tx address: " << localAddrs );
            clientApps2.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps2.Get(ac)->GetNode(),
                                                               localAddrs));
        }
        for (uint32_t ac = 0; ac < clientApps3.GetN(); ac++)
        {
            Ipv4Address localAddrs = clientApps3.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Tx address: " << localAddrs );
            clientApps3.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps3.Get(ac)->GetNode(),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Rx address: " << localAddrs );
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }
        for (uint32_t ac = 0; ac < serverApps2.GetN(); ac++)
        {
            Ipv4Address localAddrs = serverApps2.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Rx address: " << localAddrs );
            serverApps2.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps2.Get(ac)->GetNode(),
                                                               localAddrs));
        }
        for (uint32_t ac = 0; ac < serverApps3.GetN(); ac++)
        {
            Ipv4Address localAddrs = serverApps3.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            NS_LOG_INFO( "Rx address: " << localAddrs );
            serverApps3.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps3.Get(ac)->GetNode(),
                                                               localAddrs));
        }
    }
    else
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            clientApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            NS_LOG_INFO( "Tx address: " << localAddrs );
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               clientApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            serverApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            NS_LOG_INFO( "Rx address: " << localAddrs );
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               serverApps.Get(ac)->GetNode(),
                                                               localAddrs));
        }
    }

    V2xKpi v2xKpi;
    v2xKpi.SetDbPath(outputDir + exampleName);
    v2xKpi.SetTxAppDuration(txAppDurationFin);
    SavePositionPerIP(&v2xKpi);
    v2xKpi.SetRangeForV2xKpis(50);

    if (generateInitialPosGnuScript)
    {
        std::string initPosFileName = "init-pos-ues-" + exampleName + ".txt";
        PrintUeInitPosToFile(initPosFileName);
    }

    // Final simulation stop time is the addition of: simTime + slBearersActivationTime +
    // realAppStart realAppStart is of the last UE for which we installed the application
    // Time simStopTime = simTime + slBearersActivationTime + Seconds(realAppStart);
    Time simStopTime = MilliSeconds(200);

    if (generateGifGnuScript)
    {
        std::string mobilityFileName = "mobility-" + exampleName + ".txt";
        RecordMobility(true, mobilityFileName);
        WriteGifGnuScript(mobilityFileName,
                          simStopTime,
                          CAVsSpeed,
                          RsuUes.Get(0),
                          RsuUes.Get(RsuUes.GetN() - 1));
    }

    nrHelper->EnableTraces();
    
    Simulator::Stop(simStopTime);
    Simulator::Run();

    // GtkConfigStore config;
    // config.ConfigureAttributes ();

    /*
     * VERY IMPORTANT: Do not forget to empty the database cache, which would
     * dump the data store towards the end of the simulation in to a database.
     */
    //ctrlMsgsStats.EmptyCache();
    pktStats.EmptyCache();
    pscchStats.EmptyCache();
    psschStats.EmptyCache();
    pscchPhyStats.EmptyCache();
    psschPhyStats.EmptyCache();
    ueRlcRxStats.EmptyCache();
    v2xKpi.WriteKpis(numSector);

    //GtkConfigStore config;
    //config.ConfigureAttributes ();

    Simulator::Destroy();
    return 0;
}