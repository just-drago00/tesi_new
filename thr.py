import sqlite3
import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pickle
from itertools import chain
import pickleshare
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go
import ipaddress
import matplotlib.font_manager as fm


def sort_ip_index(index):
    return sorted(index, key=lambda ip: ipaddress.IPv4Address(ip))

os.makedirs('/home/justdrago/v2x/ns-3-dev/Data', exist_ok=True)

def calculate_mean_throughput(db_path):
    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Query to get the throughput data where thputKbps is not equal to 0
    query = "SELECT dstIp, thputKbps FROM thput WHERE thputKbps != 0"
    df = pd.read_sql_query(query, conn)

    # Calculate the mean throughput for each dstIp
    mean_throughput = df.groupby('dstIp')['thputKbps'].sum()

    # Close the connection
    conn.close()
    return mean_throughput


def get_mean_delay(db_path):
    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Query to get the delay data
    query = "SELECT AVG (delayMicroSec) FROM rlcRx WHERE delayMicroSec IS NOT NULL"
    df = pd.read_sql_query(query, conn)

    # Calculate the mean delay
    mean_delay = df.iloc[0, 0]

    # Close the connection
    conn.close()

    return mean_delay

def extract_delay_values(db_path):
    # Connect to the database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Query to get the delay data
    query = "SELECT delayMicroSec FROM rlcRx WHERE delayMicroSec IS NOT NULL"
    df = pd.read_sql_query(query, conn)
    df['delayMillisec'] = df['delayMicroSec'] / 1000.0
    # Close the connection
    conn.close()

    return df['delayMillisec']

def extract_sum_thput(db_path):
    conn = sqlite3.connect(db_path)

    # Query per sommare i valori di throughput per dstIp univoci
    query = """
    SELECT dstIp, SUM(thputKbps) AS total_throughput
    FROM thput
    WHERE thputKbps != 0
    GROUP BY dstIp
    """
    df = pd.read_sql_query(query, conn)
    conn.close()

    # Restituisci una Series con dstIp come indice
    return df.set_index('dstIp')['total_throughput']


def plot_mean_throughput(mean_throughput, title):
    mean_throughput.plot(kind='bar')
    plt.title(title)
    plt.xlabel('Destination IP')
    plt.ylabel('Mean Throughput (Kbps)')
    plt.show()


def plot_violin_plot(combined_df):
    plt.figure(figsize=(10, 6))
    # Ordina le categorie secondo l'ordine di apparizione
    ordered_cats = pd.Categorical(combined_df['Database'], categories=combined_df['Database'].unique(), ordered=True)
    combined_df['Database'] = ordered_cats

    custom_palette = ["pink", "yellow", "green", "violet", "black", "blue"]
    ax = sns.violinplot(x='Database', y='Delay', data=combined_df, palette=custom_palette)
    plt.title('Violin Plot of Delay Values')
    plt.ylabel('Delay (ms)')

    # Calcola la media per ogni gruppo, mantenendo l'ordine
    means = combined_df.groupby('Database', observed=True)['Delay'].mean()
    x_positions = range(len(means))
    ax.plot(x_positions, means.values, color='red', marker='o', linestyle='-', linewidth=2, label='Mean Delay')
    for i, mean in enumerate(means.values):
        ax.text(i, mean + 5, f'{mean:.2f}', ha='center', color='red')

    handles = [plt.Line2D([0], [0], color=color, lw=4) for color in custom_palette[:len(means)]]
    labels = means.index.tolist()
    plt.legend(handles + [ax.lines[-1]], labels + ['Mean Delay'], title='Database')

def plot_violino(db_paths, tags):
    delay_data = []
    labels = []
    for i, db_path in enumerate(db_paths):
        delay_values = extract_delay_values(db_path)
        if len(delay_values) > 0:
            delay_data.extend(delay_values)
            labels.extend([tags[i]] * len(delay_values))

    if len(delay_data) == 0:
        print("No delay data available for plotting.")
    else:
        # Combine the delay data into a single DataFrame
        combined_df = pd.DataFrame({'Delay': delay_data, 'Database': labels})

        # Plot different types of charts
        plot_violin_plot(combined_df)
        plt.show()

def plot_mean_throughput_lines(db_paths, tags):
    plt.figure(figsize=(10, 6))
    for db_path, tag in zip(db_paths, tags):
        mean_throughput = calculate_mean_throughput(db_path)
        # Ordina gli IP in modo naturale
        sorted_ips = sorted(mean_throughput.index, key=lambda ip: ipaddress.IPv4Address(ip))
        plt.plot(sorted_ips, mean_throughput[sorted_ips], marker='o', label=tag)
    plt.title('Mean Throughput per Destination IP')
    plt.xlabel('Destination IP')
    plt.ylabel('Mean Throughput (Kbps)')
    plt.legend(title='Database')
    plt.grid(True)
    plt.tight_layout()
    plt.show()

def plot_mean_throughput_multiple_figures(db_paths, titoli):
    # Divide i database in 3 gruppi
    db_groups = [db_paths[:2], db_paths[2:4], db_paths[4:]]  # 3 gruppi di database
    titolis = [titoli[:2], titoli[2:4], titoli[4:]]  # 3 gruppi di titoli
    for k,db_group in enumerate(db_groups):
        # Crea una nuova figura per ogni gruppo
        fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharey=True)  # 1 row, 2 columns
        fig.suptitle(f'Mean Throughput - Group {titolis[k]}', fontsize=16)

        for i, db_path in enumerate(db_group):
            mean_throughput = calculate_mean_throughput(db_path)
            labels = mean_throughput.index  # Usa gli indici di questo database
            # Primo subplot: Bar plot con il mean throughput
            axes[i].bar(labels, mean_throughput, color='skyblue')
            axes[i].set_xlabel('Destination IP')
            axes[i].set_ylabel('Mean Throughput (Kbps)')
            axes[i].tick_params(axis='x', rotation=45)
        # Regola il layout
        plt.tight_layout(rect=[0, 0, 1, 0.95])  # Lascia spazio per il titolo
        plt.show()

def plot_sum_throughput(db_paths, tag):
    db_groups = [db_paths[:2], db_paths[2:4], db_paths[4:]]  # 3 gruppi di database
    tags = [tag[:2], tag[2:4], tag[4:]]  # 3 gruppi di titoli
    for k,db_group in enumerate(db_groups):
        # Crea una nuova figura per ogni gruppo
        fig, axes = plt.subplots(1, 2, figsize=(14, 6), sharey=True)  # 1 row, 2 columns
        fig.suptitle(f'Sum Throughput - Group {tags[k]}', fontsize=16)

        for i, db_path in enumerate(db_group):
            sum_throughput = extract_sum_thput(db_path)
            mean_thput = calculate_mean_throughput(db_path)
            labels = sum_throughput.index  # Usa gli indici di questo database
            # Primo subplot: Bar plot con il mean throughput
            axes[i].bar(labels, sum_throughput, color='skyblue')
            axes[i].set_xlabel('Destination IP')
            axes[i].set_ylabel('Sum Throughput (Kbps)')
            axes[i].tick_params(axis='x', rotation=45)
            
            axes[i].plot(labels, mean_thput, color='red', marker='o', linestyle='-', linewidth=2, label='Mean Throughput')
            axes[i].legend()  # Mostra la legenda per la linea del valore medio
        # Regola il layout
        plt.tight_layout(rect=[0, 0, 1, 0.95])  # Lascia spazio per il titolo
        plt.show()



if __name__ == "__main__":
    db_pathonlysl = [
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl2cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl3cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl4cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl5cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_onlysl6cav20sec.db'
    ]
    db_paths_primo_exp = [
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_semplice.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_semplice2.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_semplice10sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_semplice20sec.db'
    ]
    db_paths_completo = [
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_semplice20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_2cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_3cav20sec.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_1fullsect20sec.db'
    ]
    db_paths = [
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_1km8X8.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_4km8X8.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_750mnum1.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_750mnum3.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_4kmnum1.db',
        '/mnt/c/Users/filod/Desktop/tesi/Risultati/db/prova_4kmnum3.db'
    ]
    tags = ["1km8X8","4km8X8","750mnum1","750mnum3","4kmnum1","4kmnum3"]
    tags_primo_exp = ["1Sec","2Sec","10Sec","20_Sec"]
    tags_completo = ["1CAV-1RSU-1UGV-1DO","2CAV-1RSU-1UGV-1DO","3CAV-1RSU-1UGV-1DO","6CAV-1RSU-1UGV-1DO"]
    tags_onlysl = ["1CAV_1RSU","2CAV_1RSU","3CAV_1RSU","4CAV_1RSU","5CAV_1RSU","6CAV_1RSU"]

    delay_data = []
    labels = []

    #plot_mean_throughput_multiple_figures(db_paths, tags)
    #plot_violino(db_paths, tags)
    plot_violino(db_pathonlysl, tags_onlysl)
    plot_violino(db_paths_primo_exp, tags_primo_exp)
    plot_violino(db_paths_completo, tags_completo)

    #plot_mean_throughput_lines(db_paths, tags)
    plot_mean_throughput_lines(db_paths_primo_exp, tags_primo_exp)
    plot_mean_throughput_lines(db_paths_completo, tags_completo)
    plot_mean_throughput_lines(db_pathonlysl, tags_onlysl)

  
    #plot_mean_throughput_multiple_figures(db_paths, tags)
    #plot_mean_throughput_multiple_figures(db_paths_primo_exp, tags_primo_exp)
    #plot_mean_throughput_multiple_figures(db_paths_completo, tags_completo)
    #plot_mean_throughput_multiple_figures(db_pathonlysl, tags_onlysl)
