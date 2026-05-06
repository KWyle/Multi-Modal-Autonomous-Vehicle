#!/usr/bin/env python3
"""Convert an ONNX model to a TensorRT engine file.

Usage examples:
  python3 model_conversion.py --onnx /path/to/model.onnx --engine /path/to/model.engine --fp16
  python3 model_conversion.py --onnx model.onnx --engine model.engine --workspace 2048
"""

import argparse
import os
import sys

try:
    import tensorrt as trt
except ImportError as exc:
    print("ERROR: TensorRT Python bindings are required to run this script.")
    print(str(exc))
    sys.exit(1)

TRT_LOGGER = trt.Logger(trt.Logger.INFO)


def parse_args():
    parser = argparse.ArgumentParser(description="Convert ONNX to TensorRT engine.")
    parser.add_argument("--onnx", required=True, help="Path to the ONNX model file.")
    parser.add_argument("--engine", required=True, help="Output TensorRT engine file path.")
    parser.add_argument("--workspace", type=int, default=2048,
                        help="Max workspace size in MiB (default: 2048).")
    parser.add_argument("--batch", type=int, default=1,
                        help="Maximum batch size for the engine (default: 1).")
    parser.add_argument("--fp16", action="store_true",
                        help="Enable FP16 precision if available.")
    parser.add_argument("--int8", action="store_true",
                        help="Enable INT8 precision if available (requires calibration).")
    parser.add_argument("--verbose", action="store_true",
                        help="Enable verbose TensorRT output.")
    parser.add_argument("--save-timing-cache", action="store_true",
                        help="Save the engine timing cache alongside the engine file.")
    parser.add_argument("--input-shape", nargs="+", type=int,
                        help="Optional static input shape as N C H W for explicit batch models.")
    return parser.parse_args()


def validate_paths(args):
    if not os.path.exists(args.onnx):
        raise FileNotFoundError(f"ONNX file not found: {args.onnx}")

    engine_dir = os.path.dirname(os.path.abspath(args.engine))
    if engine_dir and not os.path.isdir(engine_dir):
        os.makedirs(engine_dir, exist_ok=True)


def build_engine(onnx_path, engine_path, workspace_mb, max_batch, use_fp16, use_int8,
                 verbose=False, save_timing_cache=False, input_shape=None):
    if verbose:
        print(f"[convert] ONNX: {onnx_path}")
        print(f"[convert] Engine: {engine_path}")
        print(f"[convert] Workspace: {workspace_mb} MiB")
        print(f"[convert] Batch: {max_batch}")
        print(f"[convert] FP16: {use_fp16}, INT8: {use_int8}")

    builder = trt.Builder(TRT_LOGGER)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, TRT_LOGGER)

    trt.init_libnvinfer_plugins(TRT_LOGGER, "")

    with open(onnx_path, "rb") as model_file:
        model_data = model_file.read()

    if not parser.parse(model_data):
        print("ERROR: Failed to parse ONNX model. Parser errors:")
        for idx in range(parser.num_errors):
            error = parser.get_error(idx)
            print(f"  [{idx}] {error.desc()} (code={error.code()}, line={error.line()})")
        raise RuntimeError("ONNX parsing failed.")

    if input_shape is not None:
        if len(input_shape) != 4:
            raise ValueError("--input-shape must be specified as N C H W")
        if verbose:
            print(f"[convert] Forcing input shape: {input_shape}")
        for idx in range(network.num_inputs):
            tensor = network.get_input(idx)
            tensor.shape = tuple(input_shape)

    config = builder.create_builder_config()
    workspace_bytes = workspace_mb * (1 << 20)
    if hasattr(config, "max_workspace_size"):
        config.max_workspace_size = workspace_bytes
    else:
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)

    if use_fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            if verbose:
                print("[convert] FP16 enabled.")
        else:
            print("WARNING: FP16 not supported on this platform.")

    if use_int8:
        if builder.platform_has_fast_int8:
            config.set_flag(trt.BuilderFlag.INT8)
            print("WARNING: INT8 mode enabled, but no calibration is provided.")
        else:
            print("WARNING: INT8 not supported on this platform.")

    if verbose:
        config.set_flag(trt.BuilderFlag.PREFER_PRECISION_CONSTRAINTS)

    profile = builder.create_optimization_profile()
    for idx in range(network.num_inputs):
        tensor = network.get_input(idx)
        shape = tensor.shape
        if len(shape) == 4:
            min_shape = tuple(1 if d == -1 else d for d in shape)
            opt_shape = tuple(max(1, d) for d in shape)
            max_shape = tuple(max(max_batch, d if d != -1 else max_batch) for d in shape)
            profile.set_shape(tensor.name, min_shape, opt_shape, max_shape)
            config.add_optimization_profile(profile)
            if verbose:
                print(f"[convert] Optimization profile set for input '{tensor.name}':")
                print(f"          min={min_shape}, opt={opt_shape}, max={max_shape}")
        else:
            if verbose:
                print(f"[convert] Skipping optimization profile for non-4D input '{tensor.name}'.")

    if verbose:
        print("[convert] Building TensorRT engine. This can take several minutes...")

    engine = builder.build_serialized_network(network, config)
    if engine is None:
        raise RuntimeError("Engine build failed. Check ONNX model compatibility and TensorRT log output.")

    with open(engine_path, "wb") as f:
        f.write(engine)

    if save_timing_cache:
        cache_path = os.path.splitext(engine_path)[0] + ".timing.cache"
        with open(cache_path, "wb") as f:
            f.write(config.serialize())
        if verbose:
            print(f"[convert] Timing cache written to {cache_path}")

    print(f"SUCCESS: TensorRT engine saved to {engine_path}")


def main():
    args = parse_args()
    validate_paths(args)

    try:
        build_engine(
            onnx_path=args.onnx,
            engine_path=args.engine,
            workspace_mb=args.workspace,
            max_batch=args.batch,
            use_fp16=args.fp16,
            use_int8=args.int8,
            verbose=args.verbose,
            save_timing_cache=args.save_timing_cache,
            input_shape=args.input_shape,
        )
    except Exception as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)


if __name__ == "__main__":
    main()
