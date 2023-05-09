"""
@author: wangye(Wayne)
@license: Apache Licence
@file: transfer_hdf5_to_txt.py
@time: 20221116
@contact: wang121ye@hotmail.com
@site:  wangyendt@github.com
@software: PyCharm

# code is far away from bugs.
"""

import pandas as pd
import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import h5py
from pywayne.tools import *


def main():
    files = list_all_files('./data_hdf5', ['.hdf5'])
    for i, file in enumerate(files):
        with h5py.File(file, 'r') as f:
            print(f'{i=},{file=}')
            freq = float(f.attrs['sampling_rate'])
            acc = f['imu_acc'][:]
            gyro = f['imu_gyr'][:]
            mag = f['imu_mag'][:]
            movement = f['movement'][:]
            quat = f['opt_quat'][:]
            ts = np.arange(0, acc.shape[0]) / freq
            df = pd.DataFrame({
                'time': ts,
                'acc_x': acc[:, 0],
                'acc_y': acc[:, 1],
                'acc_z': acc[:, 2],
                'gyro_x': gyro[:, 0],
                'gyro_y': gyro[:, 1],
                'gyro_z': gyro[:, 2],
                'mag_x': mag[:, 0],
                'mag_y': mag[:, 1],
                'mag_z': mag[:, 2],
                'movement': movement,
                'q_w': quat[:, 0],
                'q_x': quat[:, 1],
                'q_y': quat[:, 2],
                'q_z': quat[:, 3]
            })
            print(df.head())
            txt_file = file.replace('hdf5', 'txt')
            df.to_csv(txt_file, index=False, sep=',')
            fig, ax = plt.subplots(4, 1, sharex='all')
            plt.suptitle(file)
            ax[0].plot(ts, acc)
            ax[0].plot(ts, movement * (acc.max() - acc.min()) + acc.min())
            ax[1].plot(ts, gyro)
            ax[1].plot(ts, movement * (gyro.max() - gyro.min()) + gyro.min())
            ax[2].plot(ts, mag)
            ax[2].plot(ts, movement * (mag.max() - mag.min()) + mag.min())
            ax[3].plot(ts, quat)
            ax[3].plot(ts, movement * (quat.max() - quat.min()) + quat.min())
            plt.show()
            break


if __name__ == '__main__':
    main()
