#!/usr/bin/env python3
"""Export a go2 navigation checkpoint to the ONNX policy format used by rl_nav_controller."""

from __future__ import annotations

import argparse
import os
import sys
import types
from pathlib import Path

import torch


def build_actor_critic_and_exporter():
    workspace_root = Path(__file__).resolve().parents[2]
    training_repo = workspace_root.parent / "sru-navigation-learning"
    if not training_repo.exists():
        raise FileNotFoundError(f"Training repo not found: {training_repo}")

    # The exporter only needs network definitions. Training utilities import GitPython
    # transitively, so provide a minimal stub if that optional package is absent.
    if "git" not in sys.modules:
        sys.modules["git"] = types.ModuleType("git")

    sys.path.insert(0, str(training_repo))
    from rsl_rl.modules.actor_critic_sru import (  # noqa: WPS433
        ActorCriticSRU,
        _ActorCriticSRUONNXExporterSingleCam,
    )

    # These dimensions match the go2 navigation training config and the rl_nav_controller deployment pipeline:
    # actor obs = 16 proprio + 64*5*8 depth latent = 2576
    # critic obs = 16 proprio + 64*7*7 height + 64*5*8 image + 1 time = 5713
    actor_critic = ActorCriticSRU(
        num_actor_obs=2576,
        num_critic_obs=5713,
        num_actions=3,
        actor_hidden_dims=[512, 256, 128],
        critic_hidden_dims=[512, 256, 128],
        activation="elu",
        init_noise_std=1.0,
        image_input_dims=(64, 5, 8),
        height_input_dims=(64, 7, 7),
        rnn_hidden_size=512,
        rnn_type="lstm_sru",
        rnn_num_layers=1,
        memory_variant="dual_gated",
        dropout=0.2,
        num_cameras=1,
    )
    return actor_critic, _ActorCriticSRUONNXExporterSingleCam


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        default="rl_nav_controller/deployment_policies/go2_nav_policy_randomize.pt",
        help="Path to the go2 training checkpoint.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Path to the exported ONNX policy. Defaults to the checkpoint path with an .onnx suffix.",
    )
    args = parser.parse_args()

    checkpoint_path = Path(args.checkpoint).resolve()
    output_path = (
        Path(args.output).resolve()
        if args.output is not None
        else checkpoint_path.with_suffix(".onnx")
    )

    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    if "model_state_dict" not in checkpoint:
        raise KeyError(f"Checkpoint missing 'model_state_dict': {checkpoint_path}")

    actor_critic, exporter_cls = build_actor_critic_and_exporter()
    actor_critic.load_state_dict(checkpoint["model_state_dict"], strict=True)
    actor_critic.eval()
    actor_critic.to("cpu")

    os.makedirs(output_path.parent, exist_ok=True)
    exporter = exporter_cls(
        attn_image_net=actor_critic.attn_image_net,
        memory_a=actor_critic.memory_a,
        linear_dropout_actor=actor_critic.linear_dropout_actor,
        actor=actor_critic.actor,
        image_input_dims=actor_critic.image_input_dims,
        num_image_features=actor_critic.num_image_features,
        actor_proprioceptive_input_dim=actor_critic.actor_proprioceptive_input_dim,
        normalizer=None,
    )
    exporter.eval()
    exporter.to("cpu")

    dummy_obs = torch.zeros(1, actor_critic.num_actor_obs)
    dummy_hidden = torch.zeros(actor_critic.memory_a.rnn.num_layers, 1, actor_critic.memory_a.rnn.hidden_size)
    dummy_cell = torch.zeros(actor_critic.memory_a.rnn.num_layers, 1, actor_critic.memory_a.rnn.hidden_size)

    torch.onnx.export(
        exporter,
        (dummy_obs, dummy_hidden, dummy_cell),
        str(output_path),
        input_names=["obs", "h_in", "c_in"],
        output_names=["actions", "h_out", "c_out"],
        dynamic_axes={
            "obs": {0: "batch_size"},
            "h_in": {1: "batch_size"},
            "c_in": {1: "batch_size"},
            "actions": {0: "batch_size"},
            "h_out": {1: "batch_size"},
            "c_out": {1: "batch_size"},
        },
        opset_version=17,
        do_constant_folding=True,
        dynamo=False,
    )
    print(f"Exported ONNX policy to: {output_path}")


if __name__ == "__main__":
    main()
