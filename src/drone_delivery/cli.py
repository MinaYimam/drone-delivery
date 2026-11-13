"""Command-line interface: `drone-delivery run ...`."""

from __future__ import annotations

import argparse
import json
import sys

from .scenario import load_scenario, random_scenario
from .simulation import SimulationConfig


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="drone-delivery", description="Drone delivery fleet simulator")
    sub = parser.add_subparsers(dest="command", required=True)

    run = sub.add_parser("run", help="run a simulation and print a report")
    run.add_argument("--scenario", help="path to a scenario JSON file (otherwise a random city is generated)")
    run.add_argument("--seed", type=int, default=42, help="random seed for generated scenarios")
    run.add_argument("--hubs", type=int, default=2)
    run.add_argument("--drones", type=int, default=6)
    run.add_argument("--orders", type=int, default=40)
    run.add_argument("--zones", type=int, default=2, help="number of no-fly zones")
    run.add_argument("--size", type=float, default=12.0, help="city width in km")
    run.add_argument("--tick", type=float, default=10.0, help="seconds per simulation step")
    run.add_argument("--reserve", type=float, default=0.15, help="battery reserve fraction (0-1)")
    run.add_argument("--export", metavar="FILE", help="write a JSON replay to FILE")
    run.add_argument("--json", action="store_true", help="print the report as JSON")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.command == "run":
        if args.scenario:
            scenario = load_scenario(args.scenario)
        else:
            scenario = random_scenario(
                seed=args.seed,
                n_hubs=args.hubs,
                n_drones=args.drones,
                n_orders=args.orders,
                n_zones=args.zones,
                size_km=args.size,
            )
        config = SimulationConfig(tick_seconds=args.tick, reserve_fraction=args.reserve)
        sim = scenario.build(config)
        report = sim.run()
        if args.json:
            print(json.dumps(report.to_dict(), indent=2))
        else:
            print(report.format())
        if args.export:
            sim.export(args.export)
            print(f"\nReplay written to {args.export}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
