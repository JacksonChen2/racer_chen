#!/usr/bin/env python3
"""Plot adjacent-episode change rates for completed training episodes."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics

from PIL import Image, ImageDraw, ImageFont
from tensorboard.backend.event_processing.event_accumulator import (
    EventAccumulator,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def scalar_values(accumulator: EventAccumulator, tag: str) -> list[float]:
    if tag not in accumulator.Tags()["scalars"]:
        raise KeyError(f"TensorBoard scalar {tag!r} is missing")
    return [float(item.value) for item in accumulator.Scalars(tag)]


def weighted_reward(accumulator: EventAccumulator) -> tuple[float, float, int]:
    rewards = scalar_values(accumulator, "training/reward")
    sizes = scalar_values(accumulator, "rollout/buffer_size")
    if len(rewards) != len(sizes):
        raise ValueError("reward and rollout-size scalar counts do not match")
    transitions = int(sum(sizes))
    episode_return = sum(value * size for value, size in zip(rewards, sizes))
    return episode_return / transitions, episode_return, transitions


def relative_change(current: float, previous: float) -> float:
    if abs(previous) < 1.0e-12:
        raise ZeroDivisionError("cannot calculate a relative change from zero")
    return 100.0 * (current - previous) / abs(previous)


def load_font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    paths = [
        Path("/usr/share/fonts/truetype/dejavu") / name,
        Path("/usr/share/fonts/dejavu") / name,
    ]
    for path in paths:
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.truetype(name, size=size)


def centered_text(
    draw: ImageDraw.ImageDraw,
    xy: tuple[float, float],
    text: str,
    font: ImageFont.FreeTypeFont,
    fill: str,
) -> None:
    draw.text(xy, text, font=font, fill=fill, anchor="mm")


def draw_change_plot(
    rows: list[dict[str, float | int]],
    panels: list[tuple[str, str, str, str]],
    png_path: Path,
) -> None:
    width, height = 2880, 1620
    image = Image.new("RGB", (width, height), "#f5f7fa")
    draw = ImageDraw.Draw(image)
    title_font = load_font(50, bold=True)
    subtitle_font = load_font(26)
    panel_title_font = load_font(31, bold=True)
    note_font = load_font(20)
    axis_font = load_font(19)
    value_font = load_font(22, bold=True)
    footer_font = load_font(21)

    centered_text(
        draw,
        (width / 2, 55),
        "Matched LLM Action-Prior Training: Adjacent-Episode Change Rates",
        title_font,
        "#172033",
    )
    centered_text(
        draw,
        (width / 2, 105),
        f"Completed episodes: 1-{len(rows)} (partial episodes excluded)",
        subtitle_font,
        "#526176",
    )

    outer_x, top, gap_x, gap_y = 70, 150, 35, 35
    panel_width = (width - 2 * outer_x - 2 * gap_x) / 3
    panel_height = 645
    episodes = [int(row["episode"]) for row in rows[1:]]

    for panel_index, (title, field, note, direction) in enumerate(panels):
        panel_row, panel_col = divmod(panel_index, 3)
        left = outer_x + panel_col * (panel_width + gap_x)
        upper = top + panel_row * (panel_height + gap_y)
        right, lower = left + panel_width, upper + panel_height
        draw.rounded_rectangle(
            (left, upper, right, lower),
            radius=20,
            fill="#ffffff",
            outline="#d9e0ea",
            width=2,
        )
        centered_text(
            draw,
            ((left + right) / 2, upper + 42),
            title,
            panel_title_font,
            "#172033",
        )
        centered_text(
            draw,
            ((left + right) / 2, upper + 79),
            note,
            note_font,
            "#607087",
        )

        plot_left, plot_right = left + 100, right - 35
        plot_top, plot_bottom = upper + 125, lower - 75
        values = [float(row[field]) for row in rows[1:]]
        max_abs = max(max(abs(value) for value in values) * 1.3, 0.5)

        def y_position(value: float) -> float:
            return plot_top + (max_abs - value) / (2 * max_abs) * (
                plot_bottom - plot_top
            )

        for fraction in (-1.0, -0.5, 0.0, 0.5, 1.0):
            value = fraction * max_abs
            y = y_position(value)
            color = "#344054" if fraction == 0.0 else "#e6eaf0"
            line_width = 3 if fraction == 0.0 else 1
            draw.line((plot_left, y, plot_right, y), fill=color, width=line_width)
            draw.text(
                (plot_left - 10, y),
                f"{value:+.1f}%" if value else "0%",
                font=axis_font,
                fill="#667085",
                anchor="rm",
            )

        slot_width = (plot_right - plot_left) / len(values)
        bar_width = min(130.0, slot_width * 0.55)
        zero_y = y_position(0.0)
        for index, (episode, value) in enumerate(zip(episodes, values)):
            x_center = plot_left + slot_width * (index + 0.5)
            value_y = y_position(value)
            if direction == "higher":
                color = "#2a9d8f" if value >= 0 else "#e76f51"
            elif direction == "lower":
                color = "#2a9d8f" if value <= 0 else "#e76f51"
            else:
                color = "#457b9d"
            draw.rounded_rectangle(
                (
                    x_center - bar_width / 2,
                    min(zero_y, value_y),
                    x_center + bar_width / 2,
                    max(zero_y, value_y),
                ),
                radius=9,
                fill=color,
            )
            label_y = value_y - 18 if value >= 0 else value_y + 18
            anchor = "ms" if value >= 0 else "ma"
            draw.text(
                (x_center, label_y),
                f"{value:+.2f}%",
                font=value_font,
                fill=color,
                anchor=anchor,
            )
            draw.text(
                (x_center, plot_bottom + 27),
                f"E{episode - 1} -> E{episode}",
                font=axis_font,
                fill="#475467",
                anchor="mm",
            )

    centered_text(
        draw,
        (width / 2, height - 35),
        "Rate = (current - previous) / |previous| x 100%. "
        "For negative reward, a positive rate means improvement.",
        footer_font,
        "#526176",
    )
    image.save(png_path, format="PNG", optimize=True)


def main() -> None:
    args = parse_args()
    campaign = args.campaign.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    completed = [
        directory
        for directory in sorted(campaign.glob("episode_*"))
        if (directory / "warehouse_full_bs_round_robin_result.json").is_file()
        and (directory / "training" / "training_state.json").is_file()
        and (directory / "supervisor_result.json").is_file()
    ]
    if len(completed) < 2:
        raise RuntimeError("at least two completed episodes are required")

    event_files = sorted(
        campaign.glob("**/events.out.tfevents.*"),
        key=lambda path: path.stat().st_mtime_ns,
    )
    if len(event_files) < len(completed):
        raise RuntimeError(
            f"found {len(event_files)} event files for {len(completed)} episodes"
        )

    rows: list[dict[str, float | int]] = []
    for episode, (directory, event_file) in enumerate(
        zip(completed, event_files), start=1
    ):
        result = json.loads(
            (directory / "warehouse_full_bs_round_robin_result.json").read_text()
        )
        state = json.loads(
            (directory / "training" / "training_state.json").read_text()
        )
        accumulator = EventAccumulator(str(event_file))
        accumulator.Reload()
        mean_reward, episode_return, transitions = weighted_reward(accumulator)
        communication = result["communication"]["statistics"]
        metrics = result["metrics"]
        gate_values = scalar_values(accumulator, "llm_prior/gate_mean")
        kl_values = scalar_values(
            accumulator, "llm_prior/bernoulli_product_kl_mean"
        )
        rows.append(
            {
                "episode": episode,
                "transitions": transitions,
                "mean_reward": mean_reward,
                "episode_return": episode_return,
                "joint_coverage_pct": 100.0
                * float(metrics["mapping_coverage_joint"]),
                "bs_prb": int(communication["bs_uplink_prb_slots"])
                + int(communication["bs_downlink_prb_slots"]),
                "j_c_hat": float(state["last_J_C_hat"]),
                "gate_mean": statistics.mean(gate_values),
                "action_kl_mean": statistics.mean(kl_values),
            }
        )

    rate_fields = {
        "mean_reward_change_pct": "mean_reward",
        "joint_coverage_change_pct": "joint_coverage_pct",
        "bs_prb_change_pct": "bs_prb",
        "j_c_hat_change_pct": "j_c_hat",
        "gate_change_pct": "gate_mean",
        "action_kl_change_pct": "action_kl_mean",
    }
    for index, row in enumerate(rows):
        for output_name, source_name in rate_fields.items():
            row[output_name] = (
                float("nan")
                if index == 0
                else relative_change(
                    float(row[source_name]), float(rows[index - 1][source_name])
                )
            )

    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "episode_change_rates.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    panels = [
        (
            "Mean reward",
            "mean_reward_change_pct",
            "Higher / positive is better",
            "higher",
        ),
        (
            "Joint coverage",
            "joint_coverage_change_pct",
            "Higher / positive is better",
            "higher",
        ),
        (
            "BS PRB usage",
            "bs_prb_change_pct",
            "Lower / negative is better",
            "lower",
        ),
        (
            "Task loss J_C_hat",
            "j_c_hat_change_pct",
            "Lower / negative is better",
            "lower",
        ),
        (
            "LLM-prior gate",
            "gate_change_pct",
            "Negative means weaker LLM influence",
            "neutral",
        ),
        (
            "Action-distribution KL",
            "action_kl_change_pct",
            "Negative means less distribution shift",
            "neutral",
        ),
    ]
    png_path = output_dir / "episode_change_rates.png"
    draw_change_plot(rows, panels, png_path)
    print(
        json.dumps(
            {
                "completed_episodes": len(rows),
                "csv": str(csv_path),
                "plot": str(png_path),
                "latest": rows[-1],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
