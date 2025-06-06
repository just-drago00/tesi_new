/*
 * ef-ctrl-ul-dci-stats.h
 *
 *  Created on: Mar 23, 2022
 *      Author: ugjeci
 */

#ifndef SL_CONTRIB_HELPER_CTRL_MSGS_STATS_H_
#define SL_CONTRIB_HELPER_CTRL_MSGS_STATS_H_

#include <inttypes.h>
#include <vector> 

#include <ns3/sqlite-output.h>
#include <ns3/sfnsf.h>

#include <ns3/core-module.h>
// #include <ns3/nr-control-messages.h>

namespace ns3 {

/**
 * \brief Class to collect and store the SINR values obtained from a simulation
 *
 * The class is meant to store in a database the SINR values from UE or GNB during
 * a simulation. The class contains a cache, that after some time is written
 * to the disk.
 *
 * \see SetDb
 * \see SaveSinr
 * \see EmptyCache
 */
class SlCtrlMsgsStats
{
public:
  /**
   * \brief Constructor
   */
	SlCtrlMsgsStats ();

  /**
   * \brief Install the output dabase.
   * \param db database pointer
   * \param tableName name of the table where the values will be stored
   *
   *  The db pointer must be valid through all the lifespan of the class. The
   * method creates, if not exists, a table for storing the values. The table
   * will contain the following columns:
   *
   * - "(Frame INTEGER NOT NULL, "
   * - "SubFrame INTEGER NOT NULL,"
   * - "Slot INTEGER NOT NULL,"   
   * - "CellId INTEGER NOT NULL,"
   * - "rnti INTEGER NOT NULL,"
   * - "BwpId INTEGER NOT NULL,"
   * - "MessageType Char"
   * - "Seed INTEGER NOT NULL,"
   * - "Run INTEGER NOT NULL);"
   *
   * Please note that this method, if the db already contains a table with
   * the same name, also clean existing values that has the same
   * Seed/Run pair.
   */
  void SetDb (SQLiteOutput *db, const std::string& tableName = "ctrlMsgsStats", uint32_t writeSize = 100000);

  /**
   * \brief Save the slot statistics
   * \param [in] sfnSf Slot number
   * \param [in] rnti rnti
   * \param [in] bwpId BWP ID
   * \param [in] cellId Cell ID
   */
  void SaveCtrlMsgStats (std::string layer, std::string entity, uint16_t cellId, uint16_t rnti, const SfnSf &sfnsf, uint8_t bwpid, 
		std::string msg);

  /**
   * \brief Force the cache write to disk, emptying the cache itself.
   */
  void EmptyCache ();

private:
  static void
  DeleteWhere (SQLiteOutput* p, uint32_t seed, uint32_t run, const std::string &table);

  void WriteCache ();

  struct SlCtrlMsgsCache
  {
    SfnSf sfnSf;
    Time timeInstance;
    uint16_t cellId;
    uint16_t rnti;
    uint8_t bwpId;
    std::string layer;
    std::string entity;
    std::string msg;    
  };

  SQLiteOutput *m_db;                         //!< DB pointer
  std::vector<SlCtrlMsgsCache> m_ctrlMsgsCache;         //!< Result cache
  std::string m_tableName;                    //!< Table name
  uint32_t m_writeSize;
};

} // namespace ns3



#endif /* CONTRIB_ELEPHANT_FLOW_HELPER_EF_CTRL_MSGS_STATS_H_ */
