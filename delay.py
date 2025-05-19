import sqlite3
import os
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pickle
from itertools import chain
import pickleshare
_l_simulations_scenarios = []
_l_num_ues = [76]
for _scen in ["highway"]:# ["highway", 'urban']:
    for _num_ues in _l_num_ues:
        _l_simulations_scenarios.append((_num_ues, _scen))
        _READ_FROM_OTHER_NOTEBOOK = False
_all_sims = []

try:
    len(_all_sims)
except NameError:
    _READ_FROM_OTHER_NOTEBOOK = False

if not _READ_FROM_OTHER_NOTEBOOK:
    for _sim_scen in _l_simulations_scenarios:
        _scenario = _sim_scen[1]
        _data_path_main_dir = "/mnt/c/Users/filod/Desktop/tesi/Risultati/db"
        
        # Database file names
        db_files = {
            "1km8X8": "1km8X8numerology1",
            "4km8X8": "4km8X8_numerology1",
            "4kmnum1": "4km_numerology1",
            "4kmnum3": "4km_numerology3",
            "750mnum1": "750m_numerology1",
            "750mnum3": "750m_numerology3"  
        }

        _num_ues = 78

        # Construct paths for each scenario
        db_paths = {key: os.path.join(_data_path_main_dir, f"{value}.db") for key, value in db_files.items()}

        try:
            con_1km8X8 = sqlite3.connect(db_paths["1km8X8"])
            df_pkt_1km8X8 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_1km8X8)
        except:
            df_pkt_1km8X8 = None
        try:
            con_4kmnum1 = sqlite3.connect(db_paths["4kmnum1"])
            df_pkt_4kmnum1 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_4kmnum1)
        except:
            df_pkt_4kmnum1 = None
        try:
            con_4kmnum3 = sqlite3.connect(db_paths["4kmnum3"])
            df_pkt_4kmnum3 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_4kmnum3)
        except:
            df_pkt_4kmnum3 = None
        try:
            con_750mnum1 = sqlite3.connect(db_paths["750mnum1"])
            df_pkt_750mnum1 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_750mnum1)
        except:
            df_pkt_750mnum1 = None
        try:
            con_750mnum3 = sqlite3.connect(db_paths["750mnum3"])
            df_pkt_750mnum3 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_750mnum3)
        except:
            df_pkt_750mnum3 = None
        try:
            con_4km8X8 = sqlite3.connect(db_paths["4km8X8"])
            df_pkt_4km8X8 = pd.read_sql_query('''SELECT * FROM pktTxRx''', con_4km8X8)
        except:
            df_pkt_4km8X8 = None

        _all_sims.append({
            "NumUes": _num_ues,
            "Scenario": _scenario,
            "df_pkt_4kmnum1": df_pkt_4kmnum1,
            "df_pkt_1km8X8": df_pkt_1km8X8,
            "df_pkt_4km8X8": df_pkt_4km8X8,
            "df_pkt_4kmnum3": df_pkt_4kmnum3,
            "df_pkt_750mnum1": df_pkt_750mnum1,
            "df_pkt_750mnum3": df_pkt_750mnum3
        })
len(_all_sims)
# Add tx time in dataframe
for _sim in _all_sims:
    for scen in ["1km8X8","4km8X8","4kmnum1","4kmnum3","750mnum1","750mnum3"]:
        if f"df_pkt_{scen}" in _sim.keys():
            if (_sim[f"df_pkt_{scen}"] is not None):
                _sim[f"df_pkt_{scen}"].rename(columns={'txTimeSec': 'txTimeOrigSec'}, inplace=True)
                _sim[f"df_pkt_{scen}"]['timeSec'] = np.floor((_sim[f"df_pkt_{scen}"]['timeSec']*1e6))/1e6
                _columns_l = list(filter(lambda _col: (("_x" not in _col) & ("_y" not in _col)) & ("tximsi" not in _col) & ('txTimeSec' not in _col), _sim[f"df_pkt_{scen}"].columns))
                # print(_columns_l)
                df_pkt = _sim[f"df_pkt_{scen}"][_columns_l].copy()
                # print(df_pkt.columns)
                df_pkt_tx_unique = df_pkt[df_pkt['txRx'] == 'tx'].groupby(["srcIp", "pktSeqNum"])[['timeSec', 'imsi']].first().reset_index()
                df_pkt_tx_unique.rename(columns={'timeSec': 'txTimeSec', 'imsi': "tximsi"}, inplace=True)
                df_pkt = df_pkt.merge(df_pkt_tx_unique, on=["srcIp", "pktSeqNum"], how='left')
                _sim[f"df_pkt_{scen}"] = df_pkt
_name_maps = {"1km8X8":"1 km 8x8antennas","4km8X8":"4 km 8x8antennas","4kmnum1":"4 km Numerology 1","4kmnum3":"4 km Numerology 3","750mnum1":"750m Numerology 1","750mnum3":"750 m Numerology 3",}
df_pkt_single_df_list = []
for _sim in _all_sims:
    for _scen in ["1km8X8","4km8X8","4kmnum1","4kmnum3","750mnum1","750mnum3"]:  #
        if f"df_pkt_{_scen}" in _sim.keys():
            if (_sim[f"df_pkt_{_scen}"] is not None):
                _ref_col = 'txTimeOrigSec' if _scen == "oran" else 'txTimeSec'
                _sim[f"df_pkt_{_scen}"]['delay'] = _sim[f"df_pkt_{_scen}"]['timeSec'] - _sim[f"df_pkt_{_scen}"][_ref_col]
                _sim[f"df_pkt_{_scen}"]['NumUes'] = str(_sim['NumUes'])
                _sim[f"df_pkt_{_scen}"]['sched_entity'] = _scen
                _sim[f"df_pkt_{_scen}"]['location'] = _sim['Scenario']
                df_pkt_single_df_list.append(_sim[f"df_pkt_{_scen}"])
df_pkt_single_df = pd.concat(df_pkt_single_df_list)
figDelay = go.Figure()

# colors_d = ['#EDD782', '#FFD8D1', '#F3B631', '#F57461', '#4BC4D5', '#A9DDC7', '#64A5BB', '#9693B2', '#ABAD59', '#BCA65F']
colors_d = ['#EDD782', '#FFD8D1', 'green', 'violet', 'black', 'blue', 'red', '#9693B2', '#ABAD59', '#BCA65F']

_location = "highway" # 'urban' # 

_f_time = df_pkt_single_df['timeSec']>0.0
_f_delay = df_pkt_single_df['delay'].between(0.005, 0.220)
_f_rx = df_pkt_single_df['txRx'] == 'rx'
# for _location_ind, _location in enumerate(["urban", "highway"]): #"urban", "highway"

_f_loc = df_pkt_single_df['location'] == _location
# _f_in_sim = df_pkt_single_df['in_sim']

for _scen_ind, _scen in enumerate(["1km8X8","4km8X8","4kmnum1","4kmnum3","750mnum1",'750mnum3']):
    # for _ind, _num_ues in enumerate(_l_num_ues):
    #     _sim = list(filter(lambda _elem: _elem["NumUes"] == _num_ues, _all_sims))
    #     if len(_sim)>0:
        
    _f_all =_f_delay&_f_time&_f_rx&_f_loc
    figDelay.add_trace(go.Box(
        y=df_pkt_single_df[_f_all]['delay']*1000,
        # x=[_num_ues]*df_pkt[_f_all]['delay'].shape[0],
        name= _name_maps[str(_scen)],
        # legendgroup=_location,
        # marker=dict(size=5, symbol='circle', color='green'), 
        # box=dict(facecolor='green', linewidth=2),
        # linewidth=2,
        line=dict(width=6),
        fillcolor='white',
        boxpoints=False, # no outliers
        marker_color=colors_d[2+_scen_ind], 
        boxmean=True,
        # boxmean='sd',
        # notched=True,
    ))


# Set y-axes titles
figDelay.update_layout(plot_bgcolor='rgba(0,0,0,0)')
# figThPerUser.update_traces(boxmean=True)
figDelay.update_xaxes(title=dict(text="% of CAVs", 
                            font = dict(family = 'Old Standard TT, serif', size = 40, color = 'black')),                    
                    showline=True, linewidth=2, linecolor='black', gridcolor='white', 
                tickfont = dict(family = 'Old Standard TT, serif', size = 40, color = 'black'),
                   mirror=True,)
figDelay.update_yaxes(title = dict(text="Packet delay [ms]",
                                 font = dict(family = 'Old Standard TT, serif', size = 40, color = 'black')),
                                showline=True, linewidth=2, 
                                 linecolor='black', gridcolor='lightgrey', mirror=True, gridwidth=0.5,
                              zerolinecolor='black',
                                tickfont = dict(family = 'Old Standard TT, serif', size = 40, color = 'black'),
                          # range = list(range(0,200, 20))
                          range = [0, 225],
                          dtick=25,
                        )
                
figDelay.update_layout(
    legend=dict(
        orientation="h",
        yanchor="bottom",
        y=1.02,
        xanchor="right",
        x=1,
        # font = dict(size = 24)
        font=dict(family = 'Old Standard TT, serif', size = 30, color = 'black'),
        # Scheduling mode:
        title=dict(text="", 
                   font = dict(family = 'Old Standard TT, serif', size = 30, color = 'black'))
    ), 
    height=600,
    width=800,
    boxgroupgap=0.1, boxgap=0.1,
#     yaxis_title =   "Gbps" if _v_show_inGbps else "bps",
    boxmode='group', # group together boxes of the different traces for each value of x
    # violinmode='group' # group together boxes of the different traces for each value of x
)
figDelay.write_image('/mnt/c/Users/filod/Desktop/tesi/Risultati/delay_boxplot.png'#, format="svg" 
                     )
#figDelay.show(renderer="svg")