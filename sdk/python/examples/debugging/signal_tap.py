"""See what is on the bus: every signal, its rate, who publishes it, its latest value.

Run it as is, then swap in your own app (`rf.App.from_config("app.yaml")`).
The same table is available without writing code:

    python3 -m roboforge run app.yaml --tap

    python3 signal_tap.py
"""
import roboforge as rf


def main():
    tap = rf.Tap(every=1.0, signals=["cmd/*", "state/joint", "state/pose"])     # leave `signals` out to see everything
    app = rf.App(robot="arm",
                 controllers=[rf.controller(lambda obs, t: rf.JointCommand(position=[0.5, 0.4, -0.8]), rate=20, name="target"),
                              rf.JointPD(rate=500)],
                 callbacks=[tap])
    app.run(duration=3.2)

    # The same information as data, for your own checks and assertions.
    print("\nas data:")
    for row in app.bus.describe():
        print(f"  {row['name']:<16} {row['type']:<16} published {row['count']:>5} times by {', '.join(row['sources'])}")
    pose_rate = next(r["rate"] for r in tap.rows(app) if r["name"] == "state/pose")
    print(f"\nstate/pose arrives at {pose_rate:.0f} Hz; a controller reads the latest value with obs.pose")
    print("last joint positions:", rf.summarize(app.bus.latest("state/joint")))
    app.close()


if __name__ == "__main__":
    main()
