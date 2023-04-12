import pandas as pd

df = pd.read_csv("DS1_gt.csv",header=None)

delta_df=df.diff()
delta_df=delta_df.dropna()

delta_df.to_csv("DS1_imu.csv",index=False,header=False)

print(df.head())
print(delta_df.head())