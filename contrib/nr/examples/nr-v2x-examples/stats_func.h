
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


namespace ns3{

// NS_LOG_COMPONENT_DEFINE("StatsFunction");

    /*
 * Global methods to hook trace sources from different layers of
 * the protocol stack.
 */

/**
 * \brief Method to listen the trace SlPscchScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 1-A from UE MAC.
 *
 * \param pscchStats Pointer to the UeMacPscchTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
static void
NotifySlPscchScheduling(UeMacPscchTxOutputStats* pscchStats,
                        const SlPscchUeMacStatParameters pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace SlPsschScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 2-A and data from UE MAC.
 *
 * \param psschStats Pointer to the UeMacPsschTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
static void
NotifySlPsschScheduling(UeMacPsschTxOutputStats* psschStats,
                        const SlPsschUeMacStatParameters psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
 * \brief Method to listen the trace RxPscchTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 1-A.
 *
 * \param pscchStats Pointer to the UePhyPscchRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
static void
NotifySlPscchRx(UePhyPscchRxOutputStats* pscchStats,
                const SlRxCtrlPacketTraceParams pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace RxPsschTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 2-A and data.
 *
 * \param psschStats Pointer to the UePhyPsschRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
static void
NotifySlPsschRx(UePhyPsschRxOutputStats* psschStats,
                const SlRxDataPacketTraceParams psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}




/**
 * \brief Trace sink for RxRlcPduWithTxRnti trace of NrUeMac
 * \param stats Pointer to UeRlcRxOutputStats API responsible to write the
 *        information communicated by this trace into a database.
 * \param imsi The IMSI of the UE
 * \param rnti The RNTI of the UE
 * \param txRnti The RNTI of the TX UE
 * \param lcid The logical channel id
 * \param rxPduSize The received PDU size
 * \param delay The end-to-end, i.e., from TX RLC entity to RX
 *        RLC entity, delay in Seconds.
 */
static void
NotifySlRlcPduRx(UeRlcRxOutputStats* stats,
                 uint64_t imsi,
                 uint16_t rnti,
                 uint16_t txRnti,
                 uint8_t lcid,
                 uint32_t rxPduSize,
                 double delay)
{
    stats->Save(imsi, rnti, txRnti, lcid, rxPduSize, delay);
}

/**
 * \brief Install highway mobility
 * \param numSector Total number of lanes
 * \param numCAVsPerSector number of vehicles per lane (only odd numbers)
 * \param interCAVsDist The distance between the vehicles in a same lane
 * \param interSectorDist The distance between the lanes, i.e., the distance
 *        between the vehicles of two different lanes
 * \param CAVsSpeed The CAVsSpeed of the vehicles
 * \return A node container containing the vehicle UEs with their mobility model
 *         installed
 */
static NodeContainer
InstallSLMobility(uint16_t numSector, uint16_t numCAVsPerSector,  double CAVsSpeed, int distanza)
{
    int degrees = 0;
    double radians = degrees * M_PI / 180.0;
    double radians1 = degrees * M_PI / 180.0;
    int flag = 0;
    int r = distanza;
    int r1 = 20;
   
    NodeContainer ueNodes;
    ueNodes.Create(numCAVsPerSector * numSector);

    // NS_LOG_INFO( "Total SL UEs = " << ueNodes.GetN() );

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");

    mobility.Install(ueNodes);
    
    for (int i = 0; i < numSector; i++)
    {
        if (flag % 2 == 0 ){
            ueNodes.Get(i * numCAVsPerSector)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(r * cos(radians), r * sin(radians), 0.0));
            ueNodes.Get(i * numCAVsPerSector)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, 0.0, 0.0));
            for(int j = 1 ; j < numCAVsPerSector; j++ ){
                ueNodes.Get(i * numCAVsPerSector + j)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(r * cos(radians) + r1 * cos(radians1), r * sin(radians) + r1 * sin(radians1), 0.0));
                ueNodes.Get(i * numCAVsPerSector + j)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(CAVsSpeed * cos(radians1), CAVsSpeed * sin(radians1), 0.0));
                radians1 = radians1 + (60 * M_PI / 180.0); 
            }
        }
        if  (flag % 2 == 1 ){
            ueNodes.Get(i * numCAVsPerSector)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(r * cos(radians), r * sin(radians), 30.0));
            ueNodes.Get(i * numCAVsPerSector)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, 0.0, 0.0));
            for(int j = 1 ; j < numCAVsPerSector; j++ ){
                ueNodes.Get(i * numCAVsPerSector + j)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(r * cos(radians) + r1 * cos(radians1), r * sin(radians) + r1 * sin(radians1), 0.0));
                ueNodes.Get(i * numCAVsPerSector + j)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(CAVsSpeed * cos(radians1), CAVsSpeed * sin(radians1), 0.0));
                radians1 = radians1 + (60 * M_PI / 180.0); 
            }
        } 
        radians = radians + (60 * M_PI / 180.0);
        flag++;
    }
    
    return ueNodes;
}
static double generateRandomDouble(double min, double max) {
    std::random_device rd;  // Obtain a random number from hardware
    std::mt19937 gen(rd()); // Seed the generator
    std::uniform_real_distribution<> distr(min, max); // Define the range

    return distr(gen);
}
static NodeContainer
InstallFR1Mobility(int numugv , double ugvspeed , int numoperators, double opspeed )
{
    int degrees = 0;
    double radians2 = degrees * M_PI / 180.0; 
    int r2 = 600;
    int r1 = 300;
    NodeContainer ueNodes;
    ueNodes.Create(numugv + numoperators) ;
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);

    // NS_LOG_INFO( "Total FR1 UEs = " << ueNodes.GetN() );

    for(int i = 0; i < numugv; i++){
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(r2 * cos(radians2), r2 * sin(radians2), 0.0));
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(ugvspeed * cos(radians2), ugvspeed * sin(radians2), 0.0));
        radians2 = radians2 + (30 * M_PI / 180.0);   
    }
    for (int i = numugv; i < numugv + numoperators; i++){
        double xpos = generateRandomDouble(-r1, r1);
        double ypos = generateRandomDouble(-r1, r1);
        double angle = generateRandomDouble(0.0, 360.0);
        double rad = angle * M_PI / 180.0;
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(xpos, ypos, 0.0));
        ueNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(opspeed * cos(rad), opspeed * sin(rad), 0.0)); 
    }   
    //ueNodes.Get(numugv + numoperators)->GetObject<ConstantVelocityMobilityModel>()->SetPosition(Vector(1.0, 1.0, 0.0));
    //ueNodes.Get(numugv + numoperators)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, 0.0, 0.0));
    return ueNodes;
}


/**
 * \brief Write the gnuplot script to plot the initial positions of the vehicle UEs
 * \param posFilename The name of the file, which this script needs to read to plot positions
 */
static void
WriteInitPosGnuScript(std::string posFilename)
{
    std::ofstream outFile;
    std::string filename = "gnu-script-" + posFilename;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        // NS_LOG_ERROR("Can't open file " << filename);
        return;
    }

    std::string pngFileName;
    pngFileName = posFilename.substr(0, posFilename.rfind('.'));
    outFile << "set terminal png" << std::endl;
    outFile << "set output \"" << pngFileName << ".png\"" << std::endl;
    outFile << "set style line 1 lc rgb 'black' ps 2 pt 2" << std::endl;
    outFile << "unset key" << std::endl;
    outFile << "set grid" << std::endl;
    outFile << "plot \"" << posFilename << "\" using 3:4 with points ls 1";
    outFile.close();
}

/**
 * \brief Print vehicle UEs initial position to a file
 * \param filename Name of the file in which to write the positions
 */
static void
PrintUeInitPosToFile(std::string filename)
{
    std::ofstream outFile;
    outFile.open(filename.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        // NS_LOG_ERROR("Can't open file " << filename);
        return;
    }
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << node->GetId() << " " << uedev->GetImsi() << " " << pos.x << " " << pos.y
                        << std::endl;
            }
        }
    }

    WriteInitPosGnuScript(filename);
}

/**
 * \brief Record mobility of the vehicle UEs every second
 * \param FirstWrite If this flag is true, write from scratch, otherwise, append to the file
 * \param fileName Name of the file in which to write the positions
 */
static void
RecordMobility(bool FirstWrite, std::string fileName)
{
    std::ofstream outFile;
    if (FirstWrite)
    {
        outFile.open(fileName.c_str(), std::ios_base::out);
        FirstWrite = false;
    }
    else
    {
        outFile.open(fileName.c_str(), std::ios_base::app);
        outFile << std::endl;
        outFile << std::endl;
    }

    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                outFile << Simulator::Now().GetSeconds() << " " << node->GetId() << " "
                        << uedev->GetImsi() << " " << pos.x << " " << pos.y << " " << pos.z << std::endl;
            }
        }
    }

    Simulator::Schedule(Seconds(1), &RecordMobility, FirstWrite, fileName);
}

/**
 * \brief Write a gnuplot script to generate gif of the vehicle UEs mobility
 * \param MobilityFileName The name of the file, which this script needs to read to plot positions
 * \param simTime The simulation time
 * \param CAVsSpeed The CAVsSpeed of the vehicles
 * \param firstUeNode The node pointer to the first UE in the simulation
 * \param lastUeNode The node pointer to the last UE in the simulation
 */
static void
WriteGifGnuScript(std::string MobilityFileName,
                  Time simTime,
                  double CAVsSpeed,
                  Ptr<Node> firstUeNode,
                  Ptr<Node> lastUeNode)
{
    std::ofstream outFile;
    std::string fileName = "gif-script-" + MobilityFileName;
    outFile.open(fileName.c_str(), std::ios_base::out | std::ios_base::trunc);
    if (!outFile.is_open())
    {
        // NS_LOG_ERROR("Can't open file " << fileName);
        return;
    }
    outFile << "set term gif animate delay 100" << std::endl;
    std::string gifFileName;
    gifFileName = MobilityFileName.substr(0, MobilityFileName.rfind('.'));
    outFile << "set output \"" << gifFileName << ".gif"
            << "\"" << std::endl;
    outFile << "unset key" << std::endl;
    outFile << "set grid" << std::endl;

    Vector firstNodePos = firstUeNode->GetObject<MobilityModel>()->GetPosition();
    Vector LastNodePos = lastUeNode->GetObject<MobilityModel>()->GetPosition();
    double xRangeLower = firstNodePos.x - 10.0;
    double xRangeUpper = simTime.GetSeconds() * CAVsSpeed + LastNodePos.x;
    double yRangeLower = firstNodePos.y - 10.0;
    double yRangeUpper = LastNodePos.y + 10.0;
    outFile << "set xrange [" << xRangeLower << ":" << xRangeUpper << "]" << std::endl;
    outFile << "set yrange [" << yRangeLower << ":" << yRangeUpper << "]" << std::endl;
    outFile << "do for [i=0:" << simTime.GetSeconds() - 1 << "] {plot \"" << MobilityFileName
            << "\" index i using 4:5}" << std::endl;
}

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
static void
GetSlBitmapFromString(std::string slBitMapString, std::vector<std::bitset<1>>& slBitMapVector)
{
    static std::unordered_map<std::string, uint8_t> lookupTable = {
        {"0", 0},
        {"1", 1},
    };

    std::stringstream ss(slBitMapString);
    std::string token;
    std::vector<std::string> extracted;

    while (std::getline(ss, token, '|'))
    {
        extracted.push_back(token);
    }

    for (const auto& v : extracted)
    {
        if (lookupTable.find(v) == lookupTable.end())
        {
            NS_FATAL_ERROR("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
        slBitMapVector.emplace_back(lookupTable[v] & 0x01);
    }
}

/**
 * \brief Save position of the UE as per its IP address
 * \param v2xKpi pointer to the V2xKpi API storing the IP of an UE and its position.
 */
static void
SavePositionPerIP(V2xKpi* v2xKpi)
{
    for (NodeList::Iterator it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        int nDevs = node->GetNDevices();
        for (int j = 0; j < nDevs; j++)
        {
            Ptr<NrUeNetDevice> uedev = node->GetDevice(j)->GetObject<NrUeNetDevice>();
            if (uedev)
            {
                Ptr<Ipv4L3Protocol> ipv4Protocol = node->GetObject<Ipv4L3Protocol>();
                Ipv4InterfaceAddress addresses = ipv4Protocol->GetAddress(1, 0);
                std::ostringstream ueIpv4Addr;
                ueIpv4Addr.str("");
                ueIpv4Addr << addresses.GetLocal();
                Vector pos = node->GetObject<MobilityModel>()->GetPosition();
                v2xKpi->FillPosPerIpMap(ueIpv4Addr.str(), pos);
            }
        }
    }
}

/**
 * \brief Method to listen the application level traces of type TxWithAddresses
 *        and RxWithAddresses.
 * \param stats Pointer to the UeToUePktTxRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param node The pointer to the TX or RX node
 * \param localAddrs The local IPV4 address of the node
 * \param txRx The string indicating the type of node, i.e., TX or RX
 * \param p The packet
 * \param srcAddrs The source address from the trace
 * \param dstAddrs The destination address from the trace
 * \param seqTsSizeHeader The SeqTsSizeHeader
 */

static void
UePacketTraceDb (UeToUePktTxRxOutputStats *stats, Ptr<Node> node, const Address &localAddrs,
                 std::string txRx, Ptr<const Packet> p, const Address &srcAddrs,
                 const Address &dstAddrs, const SeqTsSizeHeader &seqTsSizeHeader)
{
  uint32_t nodeId = node->GetId ();
  Ptr<NrUeNetDevice> mcUe = DynamicCast<NrUeNetDevice>(node->GetDevice (0));
  uint64_t imsi = 0;
  if(mcUe){
	imsi = mcUe->GetObject<NrUeNetDevice> ()->GetImsi ();
  }
  uint32_t seq = seqTsSizeHeader.GetSeq ();
  uint32_t pktSize = p->GetSize () + seqTsSizeHeader.GetSerializedSize ();
  Time txtime = seqTsSizeHeader.GetTs();

  stats->Save (txRx, localAddrs, nodeId, imsi, pktSize, srcAddrs, dstAddrs, seq, txtime);
}

static void 
ReportCtrlMessages(SlCtrlMsgsStats *ctrlMsgsStats, std::string layer, std::string entity, SfnSf sfn, 
											uint16_t cellId, uint16_t rnti,
                                              uint8_t bwpId, Ptr<const NrControlMessage> msg)
{
	std::string msgTypeString;
	if (msg->GetMessageType () == NrControlMessage::DL_CQI){
      msgTypeString = "DL_CQI";
    }
	else if (msg->GetMessageType () == NrControlMessage::SR){
		msgTypeString = "SR";
	}
	else if (msg->GetMessageType () == NrControlMessage::BSR){
		msgTypeString = "BSR";
	}
	else if (msg->GetMessageType () == NrControlMessage::RACH_PREAMBLE){
		msgTypeString = "RACH_PREAMBLE";
	}
	else if (msg->GetMessageType () == NrControlMessage::DL_HARQ){
		msgTypeString = "DL_HARQ";
	}
	else if (msg->GetMessageType () == NrControlMessage::MIB){
		msgTypeString = "MIB";
	}
	else if (msg->GetMessageType () == NrControlMessage::SIB1){
		msgTypeString = "SIB1";
	}
	else if (msg->GetMessageType () == NrControlMessage::RAR){
		msgTypeString = "RAR";
	}
	else if (msg->GetMessageType () == NrControlMessage::DL_DCI){
		msgTypeString = "DL_DCI";
	}
	else if (msg->GetMessageType () == NrControlMessage::UL_DCI){
		msgTypeString = "UL_DCI";
	}
	else{
		msgTypeString = "Other";
	}

	ctrlMsgsStats->SaveCtrlMsgStats(layer, entity, cellId, rnti, sfn, bwpId, msgTypeString);
}

}