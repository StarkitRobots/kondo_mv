import json
import sys
import pandas as pd
import numpy
ex = pd.read_excel('ODOMETRY/Odometry.xlsx')
motion_table = {mot:(ex.loc[ex.loc[ex['Motion'] == mot].index.values[0]].drop('Motion').to_dict()) for mot in ex["Motion"]}
for h in her:
    for e in her[h]:
        if type(her[h][e]) == numpy.int64:
            her[h][e] = int(her[h][e])
        elif type(her[h][e]) == numpy.float64:
            her[h][e] = float(her[h][e])
with open('data_motion.json', 'w') as fp:
    json.dump(her, fp)