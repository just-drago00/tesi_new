/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   Author: Marco Miozzo <marco.miozzo@cttc.es>
*           Nicola Baldo  <nbaldo@cttc.es>
*
*   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
*                Sourjya Dutta <sdutta@nyu.edu>
*                Russell Ford <russell.ford@nyu.edu>
*                Menglei Zhang <menglei@nyu.edu>
*/


#include "nr-bearer-stats-calculator.h"
#include "nr-bearer-stats-connector.h"

#include <ns3/log.h>


#include <ns3/lte-enb-rrc.h>
#include <ns3/lte-enb-net-device.h>
#include <ns3/lte-ue-rrc.h>
#include <ns3/lte-ue-net-device.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NrBearerStatsConnector");

/**
  * Less than operator for CellIdRnti, because it is used as key in map
  */
bool
operator < (const NrBearerStatsConnector::CellIdRnti& a, const NrBearerStatsConnector::CellIdRnti& b)
{
  return ( (a.cellId < b.cellId) || ( (a.cellId == b.cellId) && (a.rnti < b.rnti) ) );
}

/**
 * This structure is used as interface between trace
 * sources and NrBearerStatsCalculator. It stores
 * and provides calculators with cellId and IMSI,
 * because most trace sources do not provide it.
 */
struct NrBoundCallbackArgument : public SimpleRefCount<NrBoundCallbackArgument>
{
public:
  Ptr<NrBearerStatsBase> stats;  //!< statistics calculator
  uint64_t imsi; //!< imsi
  uint16_t cellId; //!< cellId
};

/**
 * Callback function for DL TX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 */
void
DlTxPduCallback (Ptr<NrBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize);
  arg->stats->DlTxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize);
}

/**
 * Callback function for DL RX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
DlRxPduCallback (Ptr<NrBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay, uint16_t txRnti)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize << delay << txRnti);
  arg->stats->DlRxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize, delay, txRnti);
}

/**
 * Callback function for UL TX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 */
void
UlTxPduCallback (Ptr<NrBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize);

  arg->stats->UlTxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize);
}

/**
 * Callback function for UL RX statistics for both RLC and PDCP
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
UlRxPduCallback (Ptr<NrBoundCallbackArgument> arg, std::string path,
                 uint16_t rnti, uint8_t lcid, uint32_t packetSize, uint64_t delay, uint16_t txRnti)
{
  NS_LOG_FUNCTION (path << rnti << (uint16_t)lcid << packetSize << delay << txRnti);

  arg->stats->UlRxPdu (arg->cellId, arg->imsi, rnti, lcid, packetSize, delay, txRnti);
}

/**
 * Callback function for ue rlc buffer size statistics
 * /param arg
 * /param path
 * /param rnti
 * /param lcid
 * /param packetSize
 * /param delay
 */
void
UeRlcBufferSizeCallback (Ptr<NrBoundCallbackArgument> arg, std::string path,
 uint32_t rlcBufferSize, uint32_t rlcMaxBufferSize)
{
  NS_LOG_FUNCTION (path << " rlc buff size " << rlcBufferSize << " max size " << rlcMaxBufferSize);
  arg->stats->UeRlcBufferSize (arg->cellId, arg->imsi, rlcBufferSize, rlcMaxBufferSize);
}

void 
UeRlcBufferSizeSlCallback (Ptr<NrBoundCallbackArgument> arg, std::string path, 
      uint16_t rnti, uint32_t srcLcId, uint32_t destLcId, uint32_t rlcBufferSize, uint32_t rlcMaxBufferSize){

	NS_LOG_FUNCTION (path << rnti << srcLcId << destLcId  << rlcBufferSize << rlcMaxBufferSize);
  arg->stats->UeRlcBufferSizeSl (arg->cellId, arg->imsi, rnti, srcLcId, destLcId, rlcBufferSize, rlcMaxBufferSize);
}



NrBearerStatsConnector::NrBearerStatsConnector ()
  : m_connected (false)
{
}

void
NrBearerStatsConnector::EnableRlcStats (Ptr<NrBearerStatsBase> rlcStats)
{
  m_rlcStats = rlcStats;
  EnsureConnected ();
}

void
NrBearerStatsConnector::EnablePdcpStats (Ptr<NrBearerStatsBase> pdcpStats)
{
  m_pdcpStats = pdcpStats;
  EnsureConnected ();
}

void
NrBearerStatsConnector::EnsureConnected ()
{
  NS_LOG_FUNCTION (this);
  if (!m_connected)
    {
      // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/NewUeContext",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyNewUeContextEnb, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessSuccessful",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyRandomAccessSuccessfulUe, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionReconfiguration",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyConnectionReconfigurationEnb, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionReconfiguration",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyConnectionReconfigurationUe, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyHandoverStartEnb, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyHandoverStartUe, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyHandoverEndOkEnb, this));
      // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifyHandoverEndOkUe, this));
      // modified
      // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/SlDrbCreated",
      //                  MakeBoundCallback (&NrBearerStatsConnector::NotifySlDrbCreated, this));
      Config::Connect ("/NodeList/*/DeviceList/*/nrUeRrc/SlDrbCreated",
                       MakeBoundCallback (&NrBearerStatsConnector::NotifySlDrbCreated, this));
      // end modification
      
      m_connected = true;
    }
}


void
NrBearerStatsConnector::NotifySlDrbCreated(NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint32_t srcL2Id, uint32_t dstL2Id, uint8_t lcId)
{
  // c->StoreUeManagerPath(context, cellId, rnti);
  // c->ConnectSrb0Traces (context, imsi, cellId, rnti);
  c->ConnectTracesUeSl (context, imsi, cellId, rnti);

  
}

void
NrBearerStatsConnector::ConnectTracesUeSl (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (context);
  NS_LOG_LOGIC ("expected context should match /NodeList/*/DeviceList/*/nrUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  if (m_rlcStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
    // modified

    Config::ConnectFailSafe (basePath + "/$ns3::NrSlUeRrc/DataRadioBearerMap/*/LcDataRadioBearerMap/*/LteRlc/RxPDU",
                      MakeBoundCallback (&UlRxPduCallback, arg));  
    Config::ConnectFailSafe (basePath + "/$ns3::NrSlUeRrc/DataRadioBearerMap/*/LcDataRadioBearerMap/*/LteRlc/BufferSize",
                      MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));  
    Config::ConnectFailSafe (basePath + "/$ns3::NrSlUeRrc/DataRadioBearerMap/*/LcDataRadioBearerMap/*/LteRlc/TxPDU",
                      MakeBoundCallback (&UlTxPduCallback, arg));   

    Config::ConnectFailSafe (basePath + "/$ns3::NrSlUeRrc/DataRadioBearerMap/*/LcDataRadioBearerMap/*/LtePdcp/RxPDU",
                      MakeBoundCallback (&UlRxPduCallback, arg));  
    Config::ConnectFailSafe (basePath + "/$ns3::NrSlUeRrc/DataRadioBearerMap/*/LcDataRadioBearerMap/*/LtePdcp/TxPDU",
                      MakeBoundCallback (&UlTxPduCallback, arg));   
    // end modification

  }
}



void
NrBearerStatsConnector::NotifyRandomAccessSuccessfulUe (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSrb0Traces (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyConnectionSetupUe (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectSrb1TracesUe (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyConnectionReconfigurationUe (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesUeIfFirstTime (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyHandoverStartUe (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  c->DisconnectTracesUe (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyHandoverEndOkUe (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesUe (context, imsi, cellId, rnti);
}



void
NrBearerStatsConnector::NotifyNewUeContextEnb (NrBearerStatsConnector* c, std::string context, uint16_t cellId, uint16_t rnti)
{
  c->StoreUeManagerPath (context, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyConnectionReconfigurationEnb (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesEnbIfFirstTime (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyHandoverStartEnb (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti, uint16_t targetCellId)
{
  c->DisconnectTracesEnb (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::NotifyHandoverEndOkEnb (NrBearerStatsConnector* c, std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  c->ConnectTracesEnb (context, imsi, cellId, rnti);
}

void
NrBearerStatsConnector::StoreUeManagerPath (std::string context, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context << cellId << rnti);
  std::ostringstream ueManagerPath;
  ueManagerPath <<  context.substr (0, context.rfind ("/")) << "/UeMap/" << (uint32_t) rnti;
  CellIdRnti key;
  key.cellId = cellId;
  key.rnti = rnti;
  m_ueManagerPathByCellIdRnti[key] = ueManagerPath.str ();
}

void
NrBearerStatsConnector::ConnectSrb0Traces (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  std::string ueRrcPath =  context.substr (0, context.rfind ("/"));
  CellIdRnti key;
  key.cellId = cellId;
  key.rnti = rnti;
  std::map<CellIdRnti, std::string>::iterator it = m_ueManagerPathByCellIdRnti.find (key);
  NS_ASSERT (it != m_ueManagerPathByCellIdRnti.end ());
  std::string ueManagerPath = it->second;
  NS_LOG_LOGIC (this << " ueManagerPath: " << ueManagerPath);
  m_ueManagerPathByCellIdRnti.erase (it);

  if (m_rlcStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;

      // diconnect eventually previously connected SRB0 both at UE and eNB
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                          MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                          MakeBoundCallback (&DlRxPduCallback, arg));
      // modified
      // the buffer size of ue rlc
      Config::Disconnect (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                          MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));
      // end modification
      Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                          MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Disconnect (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                          MakeBoundCallback (&UlRxPduCallback, arg));

      // connect SRB0 both at UE and eNB
      Config::Connect (ueRrcPath + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Connect (ueRrcPath + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      // modified
      Config::Connect (ueRrcPath + "/Srb0/LteRlc/BufferSize",
                          MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));
      // end modification
      Config::Connect (ueManagerPath + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (ueManagerPath + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));

      // connect SRB1 at eNB only (at UE SRB1 will be setup later)
      Config::Connect (ueManagerPath + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (ueManagerPath + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
    }
  if (m_pdcpStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;

      // connect SRB1 at eNB only (at UE SRB1 will be setup later)
      Config::Connect (ueManagerPath + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::Connect (ueManagerPath + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
    }
}

void
NrBearerStatsConnector::ConnectSrb1TracesUe (std::string ueRrcPath, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << imsi << cellId << rnti);
  if (m_rlcStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::Connect (ueRrcPath + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Connect (ueRrcPath + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
    }
  if (m_pdcpStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::Connect (ueRrcPath + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      Config::Connect (ueRrcPath + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
    }
}

void
NrBearerStatsConnector::ConnectTracesUeIfFirstTime (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  if (m_imsiSeenUe.find (imsi) == m_imsiSeenUe.end ())
    {
      m_imsiSeenUe.insert (imsi);
      ConnectTracesUe (context, imsi, cellId, rnti);
    }
}

void
NrBearerStatsConnector::ConnectTracesEnbIfFirstTime (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  if (m_imsiSeenEnb.find (imsi) == m_imsiSeenEnb.end ())
    {
      m_imsiSeenEnb.insert (imsi);
      ConnectTracesEnb (context, imsi, cellId, rnti);
    }
}

void
NrBearerStatsConnector::ConnectTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context should match /NodeList/*/DeviceList/*/LteUeRrc/");
  std::string basePath = context.substr (0, context.rfind ("/"));
  if (m_rlcStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::Connect (basePath + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Connect (basePath + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      Config::Connect (basePath + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Connect (basePath + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      // modified
      Config::Connect (basePath + "/DataRadioBearerMap/*/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));
      Config::Connect (basePath + "/Srb0/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));
      Config::Connect (basePath + "/Srb1/LteRlc/BufferSize",
                       MakeBoundCallback (&UeRlcBufferSizeSlCallback, arg));
      // end modification

    }
  if (m_pdcpStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::Connect (basePath + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      Config::Connect (basePath + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
      Config::Connect (basePath + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&DlRxPduCallback, arg));
      Config::Connect (basePath + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&UlTxPduCallback, arg));
    }
}

void
NrBearerStatsConnector::ConnectTracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this << context);
  NS_LOG_LOGIC (this << "expected context  should match /NodeList/*/DeviceList/*/LteEnbRrc/");
  std::ostringstream basePath;
  basePath <<  context.substr (0, context.rfind ("/")) << "/UeMap/" << (uint32_t) rnti;
  if (m_rlcStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_rlcStats;
      Config::Connect (basePath.str () + "/DataRadioBearerMap/*/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::Connect (basePath.str () + "/DataRadioBearerMap/*/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb0/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb0/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb1/LteRlc/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb1/LteRlc/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
    }
  if (m_pdcpStats)
    {
      Ptr<NrBoundCallbackArgument> arg = Create<NrBoundCallbackArgument> ();
      arg->imsi = imsi;
      arg->cellId = cellId;
      arg->stats = m_pdcpStats;
      Config::Connect (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (basePath.str () + "/DataRadioBearerMap/*/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb1/LtePdcp/TxPDU",
                       MakeBoundCallback (&DlTxPduCallback, arg));
      Config::Connect (basePath.str () + "/Srb1/LtePdcp/RxPDU",
                       MakeBoundCallback (&UlRxPduCallback, arg));
    }
}

void
NrBearerStatsConnector::DisconnectTracesUe (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
}


void
NrBearerStatsConnector::DisconnectTracesEnb (std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
  NS_LOG_FUNCTION (this);
}


Ptr<NrBearerStatsBase>
NrBearerStatsConnector::GetRlcStats ()
{
  return m_rlcStats;
}

Ptr<NrBearerStatsBase>
NrBearerStatsConnector::GetPdcpStats ()
{
  return m_pdcpStats;
}


} // namespace ns3
