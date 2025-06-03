import pandas as pd

# Load raw donkey path CSV (x, y, throttle)
df = pd.read_csv("inputs/tracks/donkey_path.csv", header=None, names=["x", "y", "throttle"])

# Set uniform track width (0.75 m to each side)
df["w_tr_right_m"] = 0.75
df["w_tr_left_m"] = 0.75

# Select only required columns in proper order
df_out = df[["x", "y", "w_tr_right_m", "w_tr_left_m"]]

# Save with commented header
with open("inputs/tracks/donkey_converted_track.csv", "w") as f:
    f.write("# x_m,y_m,w_tr_right_m,w_tr_left_m\n")
    df_out.to_csv(f, index=False, header=False)
