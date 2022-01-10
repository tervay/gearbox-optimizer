from ilite import calc
from tqdm import tqdm
import plotly.graph_objects as go

ratio = 6.0
wheel = 4
motors = 4
curr_limit = 80
sprint_dist = 12
rint = 0.015
extra_weight = 14 + 10
efficiency = 0.95 * 0.95

x = []
y = []
for w in tqdm(range(80, 126)):
    data = calc(
        ratio,
        wheel,
        motors,
        curr_limit,
        "NEO",
        sprint_dist,
        rint,
        w,
        extra_weight,
        efficiency,
    )
    x.append(w)
    y.append(data["tractive force"])

fig = go.Figure()
fig.add_trace(
    go.Scatter(
        x=x,
        y=y,
    )
)

fig.show()
