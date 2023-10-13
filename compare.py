from pprint import pformat, pprint

import plotly.graph_objects as go
from rich.console import Console
from rich.table import Table
from tqdm import tqdm

from ilite import calc, efficiency_calc, nearest_multiple


def calculate_gear_ratio(gearbox_config):
    if not gearbox_config:
        return None

    gear_ratio = 1.0
    for gear_pair in gearbox_config:
        if len(gear_pair) != 2:
            print("Invalid gear pair:", gear_pair)
            return None

        first_gear_teeth = gear_pair[0]
        second_gear_teeth = gear_pair[1]

        if second_gear_teeth == 0:
            print("Gear teeth count cannot be zero:", gear_pair)
            return None

        gear_ratio *= first_gear_teeth / second_gear_teeth

    return gear_ratio


class Gearbox:
    def __init__(self, name, motor, ratios) -> None:
        self.name = name
        self.motor = motor
        self.ratios = ratios


pinions = {
    "Kraken X60 (FOC)": [12, 13, 14],
    "NEO Vortex": [13, 14],
    "NEO": [14],
}

modules = {
    "MK4i_L3": [
        [None, 50],
        [28, 16],
        [15, 45],
    ],
}

gearboxes = []
for motor_name, motor_pinions in pinions.items():
    for pinion in motor_pinions:
        gearboxes.append(
            Gearbox(
                name=f"{motor_name}_{pinion}T",
                motor=motor_name,
                ratios=[
                    [pinion, 50],
                    [28, 16],
                    [15, 45],
                ],
            )
        )

options = []
for gb in gearboxes:
    for cl in range(30, 90, 2):
        ilite_data = calc(
            ratio_=1 / calculate_gear_ratio(gb.ratios),
            wheel_diameter_=4,
            num_motors_=4,
            current_limit_=cl,
            motor_=gb.motor,
            sprint_distance_=20,
            rint_=0.015,
            weight_=125,
            weight_auxilliary_=0,
            efficiency_=efficiency_calc(calculate_gear_ratio([gb.ratios[0]])),
        )

        options.append(
            {
                "gearbox": gb.name,
                "total reduction": round(1 / calculate_gear_ratio(gb.ratios), 2),
                "current limit": cl,
                "motors": 4,
                "motor": gb.motor,
                "wheel size": 4,
                "tractive force": round(ilite_data["tractive force"]),
                "pushing current": round(ilite_data["pushing current"]),
                "turning current": round(ilite_data["turning current"]),
                "time to goal": ilite_data["time to goal"],
                "max speed": nearest_multiple(ilite_data["max speed"], 100),
                "avg speed": ilite_data["avg speed"],
                "min voltage": nearest_multiple(ilite_data["min voltage"], 100),
                "ttgPerForce": nearest_multiple(
                    ilite_data["time to goal"] / ilite_data["tractive force"], 10000
                ),
                "push voltage": round(ilite_data["push voltage"]),
                "efficiency": round(
                    efficiency_calc(calculate_gear_ratio([gb.ratios[0]])), 3
                ),
            }
        )

options = [dict(t) for t in {tuple(d.items()) for d in options}]
sorted_options = sorted(
    options,
    key=lambda t: (
        -t["ttgPerForce"],
        -t["time to goal"],
        t["tractive force"],
        t["min voltage"],
        -t["pushing current"],
        -t["turning current"],
        t["max speed"],
    ),
    reverse=True,
)


cleaned_options = []
seen = set()
for option in tqdm(sorted(options, key=lambda o: -o["current limit"])):
    found = False
    if (
        option["gearbox"],
        option["motors"],
        option["wheel size"],
    ) in seen:
        found = True

    cleaned_options.append(option)
    seen.add(
        (
            option["gearbox"],
            option["motors"],
            option["wheel size"],
        )
    )

sorted_options = sorted(
    cleaned_options,
    key=lambda t: (
        -t["ttgPerForce"],
        -t["time to goal"],
        t["tractive force"],
        t["min voltage"],
        -t["pushing current"],
        -t["turning current"],
        t["max speed"],
    ),
    reverse=True,
)

csv = [[]]
table = Table(show_header=True, width=200)
for col in [
    "Name",
    "Motors",
    "WheelSize",
    "CurrLim",
    "TTG/TractForce",
    "TTG",
    "TractForce",
    "MinVolt",
    "PushCurr",
    "TurnCurr",
    "MaxSpd",
    # "Stages",
    "Ratio",
]:
    table.add_column(col, width={"Name": 47, "Stages": 25}.get(col, None))
    csv[0].append(col)

for opt in tqdm(sorted_options):
    to_add = ()

    table.add_row(
        opt["gearbox"],
        str(opt["motors"]),
        str(opt["wheel size"]),
        str(opt["current limit"]),
        str(opt["ttgPerForce"]),
        str(opt["time to goal"]),
        str(opt["tractive force"]),
        str(opt["min voltage"]),
        str(opt["pushing current"]),
        str(opt["turning current"]),
        str(opt["max speed"]),
        str(opt["total reduction"]),
    )
    csv.append(
        [
            opt["gearbox"].replace(",", " /"),
            str(opt["motors"]),
            str(opt["wheel size"]),
            str(opt["current limit"]),
            str(opt["ttgPerForce"]),
            str(opt["time to goal"]),
            str(opt["tractive force"]),
            str(opt["min voltage"]),
            str(opt["pushing current"]),
            str(opt["turning current"]),
            str(opt["max speed"]),
            str(opt["total reduction"]),
        ]
    )

with open(f"data.txt", "w+") as f:
    console = Console(file=f, width=200)
    console.print(table)


def graph(cleaned_options):
    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=[o["time to goal"] for o in cleaned_options],
            y=[o["tractive force"] for o in cleaned_options],
            text=[pformat(o).replace("\n", "<br>") for o in cleaned_options],
            name="6 Motors",
            marker_color=[
                {
                    "NEO Vortex": "orange",
                    "Kraken X60 (FOC)": "blue",
                    "NEO": "black",
                }[o["motor"]]
                for o in cleaned_options
            ],
            mode="markers",
        )
    )

    fig.update_yaxes(rangemode="tozero")
    fig.update_xaxes(rangemode="tozero")
    fig.update_layout(
        xaxis_title="Time to Goal (s) (Rounded to nearest 0.05s)",
        yaxis_title="Tractive Force (lbs) (Rounded to nearest 1lbs)",
        title=f"Tractive Force vs Time to Goal in MK4i L3s with varying pinion sizes",
    )

    fig.show()


graph(cleaned_options)
