#!/usr/bin/env python3

import argparse
import csv
import matplotlib.pyplot as plt


DEFAULT_FILE = "motor_logs.log"


def load_data(filename):
    current = []
    target = []
    remaining = []

    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)

        required_columns = {"current", "target", "remaining"}

        if not required_columns.issubset(reader.fieldnames or []):
            raise ValueError(
                f"Soubor musí obsahovat sloupce: "
                f"{', '.join(sorted(required_columns))}"
            )

        for row in reader:
            current.append(int(row["current"]))
            target.append(int(row["target"]))
            remaining.append(int(row["remaining"]))

    return current, target, remaining


def main():
    parser = argparse.ArgumentParser(
        description="Interaktivní graf motorových logů"
    )

    parser.add_argument(
        "filename",
        nargs="?",
        default=DEFAULT_FILE,
        help=f"Log soubor (default: {DEFAULT_FILE})",
    )

    args = parser.parse_args()

    current, target, remaining = load_data(args.filename)

    fig, ax = plt.subplots(figsize=(12, 7))

    line, = ax.plot(
        current,
        remaining,
        marker="o",
        linewidth=2,
        markersize=7,
        label="remaining",
    )

    ax.set_title(f"Motor log: {args.filename}")
    ax.set_xlabel("Current")
    ax.set_ylabel("Remaining")

    ax.grid(True, alpha=0.3)
    ax.legend()

    # Tooltip
    annotation = ax.annotate(
        "",
        xy=(0, 0),
        xytext=(15, 15),
        textcoords="offset points",
        bbox=dict(
            boxstyle="round",
            fc="white",
            ec="gray",
        ),
        arrowprops=dict(arrowstyle="->"),
    )

    annotation.set_visible(False)

    def update_annotation(event):
        if event.inaxes != ax:
            annotation.set_visible(False)
            fig.canvas.draw_idle()
            return

        contains, info = line.contains(event)

        if contains:
            index = info["ind"][0]

            annotation.xy = (
                current[index],
                remaining[index],
            )

            annotation.set_text(
                f"current:   {current[index]:,}\n"
                f"target:    {target[index]:,}\n"
                f"remaining: {remaining[index]:,}"
            )

            annotation.set_visible(True)
            fig.canvas.draw_idle()

        else:
            annotation.set_visible(False)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect(
        "motion_notify_event",
        update_annotation,
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
