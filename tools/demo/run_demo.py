"""
Drive the C++ processing pipeline via ctypes using DUNEX microSWIFT data.

Dataset citation:
    Rainville, E. J.; Thomson, Jim; Moulton, Melissa; Derakhti, Morteza (2023).
    Measurements of nearshore waves through coherent arrays of free-drifting
    wave buoys [Dataset]. Dryad. https://doi.org/10.5061/dryad.hx3ffbgk0

Loads demo_api.h as a shared library (libsmartfin_demo.dylib / .so), converts
DUNEX NetCDF IMU data to numpy arrays, and calls each pipeline stage directly.

Usage:
    python tools/demo/run_demo.py                        # mission 34
    python tools/demo/run_demo.py --mission 2
    python tools/demo/run_demo.py --local data/mission_34.nc
    python tools/demo/run_demo.py --lib path/to/libsmartfin_demo.dylib
    python tools/demo/run_demo.py --no-plot              # print only
"""

import argparse
import atexit
import builtins
import ctypes
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import xarray as xr
from scipy import signal as sp_signal

REPO_ROOT  = Path(__file__).parent.parent.parent
BUILD_DIRS = [
    REPO_ROOT / "build",
    REPO_ROOT / "cmake-build-debug",
    REPO_ROOT / "cmake-build-release",
]
LIB_NAMES = ["libsmartfin_demo.dylib", "libsmartfin_demo.so"]

THREDDS_ODAP = (
    "https://chldata.erdc.dren.mil/thredds/dodsC"
    "/frf/projects/Dunex/UW_drifters"
)

# ctypes array helpers
f32p = np.ctypeslib.ndpointer(dtype=np.float32, flags="C_CONTIGUOUS")
f64p = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
u32p = np.ctypeslib.ndpointer(dtype=np.uint32,  flags="C_CONTIGUOUS")


def tee_prints_to(log_path: Path):
    """Mirror stdout print calls to a log file."""
    log_file = log_path.open("w", encoding="utf-8")
    original_print = builtins.print

    def tee_print(*args, **kwargs):
        original_print(*args, **kwargs)

        stream = kwargs.get("file")
        if stream not in (None, sys.stdout):
            return

        log_kwargs = dict(kwargs)
        log_kwargs["file"] = log_file
        original_print(*args, **log_kwargs)
        log_file.flush()

    builtins.print = tee_print
    return original_print, log_file


def load_lib(path: Path | None) -> ctypes.CDLL:
    if path:
        lib = ctypes.CDLL(str(path))
    else:
        for d in BUILD_DIRS:
            for name in LIB_NAMES:
                candidate = d / name
                if candidate.exists():
                    lib = ctypes.CDLL(str(candidate))
                    break
            else:
                continue
            break
        else:
            raise FileNotFoundError(
                "libsmartfin_demo not found. Build it first:\n"
                "  cmake -B build && cmake --build build --target smartfin_demo"
            )

    lib.sf_demo_nperseg.restype  = ctypes.c_int
    lib.sf_demo_nperseg.argtypes = []

    lib.sf_demo_orient.restype  = ctypes.c_int
    lib.sf_demo_orient.argtypes = [
        u32p, f32p, f32p, f32p,
        ctypes.c_int,
        u32p, f64p, f64p,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
    ]

    lib.sf_demo_decimate.restype  = ctypes.c_int
    lib.sf_demo_decimate.argtypes = [
        u32p, f64p, ctypes.c_int,
        u32p, f64p,
    ]

    lib.sf_demo_bandpass.restype  = ctypes.c_int
    lib.sf_demo_bandpass.argtypes = [u32p, f64p, ctypes.c_int]

    lib.sf_demo_fft.restype  = None
    lib.sf_demo_fft.argtypes = [f64p, f64p]

    lib.sf_demo_real_dft_mag_sq.restype  = None
    lib.sf_demo_real_dft_mag_sq.argtypes = [f64p, f64p]

    lib.sf_demo_welch.restype  = None
    lib.sf_demo_welch.argtypes = [f64p, ctypes.c_int, ctypes.c_double, f64p]

    return lib


def load_mission(source: int | Path) -> xr.Dataset:
    """Load a DUNEX microSWIFT mission dataset.

    Data source:
        Rainville, E. J.; Thomson, Jim; Moulton, Melissa; Derakhti, Morteza (2023).
        Measurements of nearshore waves through coherent arrays of free-drifting
        wave buoys [Dataset]. Dryad. https://doi.org/10.5061/dryad.hx3ffbgk0
    """
    if isinstance(source, Path):
        return xr.open_dataset(source)
    return xr.open_dataset(f"{THREDDS_ODAP}/mission_{source}.nc")


def best_trajectory(ds: xr.Dataset) -> xr.Dataset:
    eta = ds["sea_surface_elevation"]
    valid = (~np.isnan(eta)).sum(dim="time")
    return ds.isel(trajectory=int(np.argmax(valid.values)))


def imu_arrays(buoy: xr.Dataset):
    t0 = buoy["time"].values[0]
    elapsed_ms = ((buoy["time"].values - t0) / np.timedelta64(1, "ms")).astype(np.uint32)

    accel = np.column_stack([
        buoy["acceleration_x_body"].values,
        buoy["acceleration_y_body"].values,
        buoy["acceleration_z_body"].values,
    ]).astype(np.float32)
    gyro = np.column_stack([
        buoy["rotation_rate_x"].values,
        buoy["rotation_rate_y"].values,
        buoy["rotation_rate_z"].values,
    ]).astype(np.float32)
    mag = np.column_stack([
        buoy["magnetic_flux_density_x"].values,
        buoy["magnetic_flux_density_y"].values,
        buoy["magnetic_flux_density_z"].values,
    ]).astype(np.float32)

    valid = (
        np.all(np.isfinite(accel), axis=1) &
        np.all(np.isfinite(gyro),  axis=1) &
        np.all(np.isfinite(mag),   axis=1)
    )
    return (
        np.ascontiguousarray(elapsed_ms[valid]),
        np.ascontiguousarray(accel[valid]),
        np.ascontiguousarray(gyro[valid]),
        np.ascontiguousarray(mag[valid]),
    )


def quat_to_euler_deg(q: np.ndarray) -> np.ndarray:
    """Convert quaternions (N,4) [w,x,y,z] to Euler angles (N,3) [roll,pitch,yaw] in degrees."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll  = np.degrees(np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y - z*x), -1, 1)))
    yaw   = np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return np.column_stack([roll, pitch, yaw])


def section(title: str) -> None:
    print(f"\n{title}")
    print("-" * len(title))


def report_accel(elapsed_ms: np.ndarray, accel: np.ndarray) -> None:
    n = len(elapsed_ms)
    if n < 2:
        print(f"  {n} samples")
        return
    fs = 1000.0 / float(np.median(np.diff(elapsed_ms.astype(np.float64))))
    az = accel[:, 2]
    print(f"  samples : {n}   fs : {fs:.2f} Hz")
    print(f"  accel_z : mean={az.mean():.4f}  std={az.std():.4f}  "
          f"[{az.min():.4f}, {az.max():.4f}]  m/s^2")


def report_ahrs_usage(n_oriented: int, accel_used: int,
                      mag_used: int, bias_used: int) -> None:
    if n_oriented <= 0:
        print("  AHRS feedback: no accepted samples")
        return

    accel_pct = 100.0 * accel_used / n_oriented
    mag_pct = 100.0 * mag_used / n_oriented
    bias_pct = 100.0 * bias_used / n_oriented
    print(f"  AHRS feedback: accel={accel_pct:5.1f}%  "
          f"mag={mag_pct:5.1f}%  gyro_bias={bias_pct:5.1f}%")

    if mag_pct < 10.0:
        print("  WARNING: magnetometer feedback rarely used; "
              "yaw/heading is weakly constrained")
    if accel_pct < 50.0:
        print("  WARNING: accelerometer feedback used less than half the time; "
              "tilt/gravity removal may be unreliable")


def _save(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=150, bbox_inches="tight")


def _subplot_fig(ax_fn, title: str, figsize=(9, 4)) -> plt.Figure:
    fig, ax = plt.subplots(figsize=figsize)
    ax_fn(ax)
    ax.grid(True, lw=0.3)
    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    return fig


def plot_pipeline(elapsed_oriented, accel_oriented,
                  dec_elapsed, dec_accel,
                  filt_accel, mission_label: str,
                  out_dir: Path) -> None:
    """Figure 1: time-domain pipeline (2x2) + individual subplots."""
    t_ori = elapsed_oriented / 1000.0
    t_dec = dec_elapsed / 1000.0

    kw_ori  = dict(lw=0.6, color="steelblue")
    kw_dec  = dict(lw=0.8, color="darkorange")
    kw_filt = dict(lw=0.9, color="seagreen")

    def draw_oriented(ax):
        ax.plot(t_ori, accel_oriented[:, 2], **kw_ori)
        ax.set_title("Oriented (post-AHRS)")
        ax.set_ylabel("accel_z  (m/s^2)")
        ax.set_xlabel("time  (s)")

    def draw_decimated(ax):
        ax.plot(t_dec, dec_accel[:, 2], **kw_dec)
        ax.set_title("Decimated")
        ax.set_ylabel("accel_z  (m/s^2)")
        ax.set_xlabel("time  (s)")

    def draw_filtered(ax):
        ax.plot(t_dec, filt_accel[:, 2], **kw_filt)
        ax.set_title("Decimated + bandpass  (0.05-0.50 Hz)")
        ax.set_ylabel("accel_z  (m/s^2)")
        ax.set_xlabel("time  (s)")

    def draw_all(ax):
        ax.plot(t_ori, accel_oriented[:, 2], label="oriented",  **kw_ori)
        ax.plot(t_dec, dec_accel[:, 2],      label="decimated", **kw_dec)
        ax.plot(t_dec, filt_accel[:, 2],     label="filtered",  **kw_filt)
        ax.set_title("All stages")
        ax.set_xlabel("time  (s)")
        ax.legend(fontsize=8)

    # Individual subplots
    _save(_subplot_fig(draw_oriented,  f"Oriented ({mission_label})"),
          out_dir / "pipeline_1_oriented.png")
    _save(_subplot_fig(draw_decimated, f"Decimated ({mission_label})"),
          out_dir / "pipeline_2_decimated.png")
    _save(_subplot_fig(draw_filtered,  f"Filtered ({mission_label})"),
          out_dir / "pipeline_3_filtered.png")
    _save(_subplot_fig(draw_all,       f"All stages ({mission_label})"),
          out_dir / "pipeline_4_all.png")

    # Combined 2x2
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    fig.suptitle(f"Pipeline — time domain  ({mission_label})", fontsize=13)
    draw_oriented(axes[0, 0])
    draw_decimated(axes[0, 1])
    draw_filtered(axes[1, 0])
    draw_all(axes[1, 1])
    for ax in axes.flat:
        ax.grid(True, lw=0.3)
    fig.tight_layout()
    _save(fig, out_dir / "pipeline_combined.png")


def plot_frequency(X: np.ndarray, X_pre: np.ndarray,
                   X_filt_all: list, welch_psd_z: np.ndarray,
                   fs_dec: float, nperseg: int,
                   mission_label: str, out_dir: Path) -> None:
    """Figure 2: frequency-domain (1x3) + individual subplots."""
    n_bins = nperseg // 2 + 1
    df     = fs_dec / nperseg
    freqs  = np.arange(n_bins) * df
    fft_mag     = np.abs(X[:n_bins])
    fft_mag_pre = np.abs(X_pre[:n_bins])
    wave_lo, wave_hi = 0.05, 0.50

    def draw_fft(ax):
        ax.plot(freqs, fft_mag_pre, lw=0.9, color="tomato",    label="pre-filter")
        ax.plot(freqs, fft_mag,     lw=0.9, color="royalblue", label="filtered")
        ax.axvspan(wave_lo, wave_hi, alpha=0.12, color="gold", label="wave band")
        ax.set_title("FFT magnitude  |X[k]|  (pre-filter vs filtered)")
        ax.set_xlabel("frequency  (Hz)")
        ax.set_ylabel("|X[k]|")
        ax.set_xlim(0, fs_dec / 2)
        ax.legend(fontsize=8)

    def draw_welch_psd(ax):
        ax.semilogy(freqs, welch_psd_z, lw=0.9, color="crimson")
        ax.axvspan(wave_lo, wave_hi, alpha=0.12, color="gold", label="wave band")
        ax.set_title("Welch PSD  (averaged)  accel_z")
        ax.set_xlabel("frequency  (Hz)")
        ax.set_ylabel("PSD  ((m/s^2)^2/Hz)")
        ax.set_xlim(0, fs_dec / 2)
        ax.legend(fontsize=8)

    def draw_xyz(ax):
        colors = ("steelblue", "darkorange", "seagreen")
        labels = ("x", "y", "z")
        for i, (c, lbl) in enumerate(zip(colors, labels)):
            ax.plot(freqs, np.abs(X_filt_all[i][:n_bins]), lw=0.9,
                    color=c, label=lbl)
        ax.axvspan(wave_lo, wave_hi, alpha=0.12, color="gold", label="wave band")
        ax.set_title("FFT magnitude by axis  (filtered)")
        ax.set_xlabel("frequency  (Hz)")
        ax.set_ylabel("|X[k]|")
        ax.set_xlim(0, fs_dec / 2)
        ax.legend(fontsize=8)

    _save(_subplot_fig(draw_fft,       f"FFT magnitude ({mission_label})"),
          out_dir / "freq_1_fft_magnitude.png")
    _save(_subplot_fig(draw_welch_psd, f"Welch PSD ({mission_label})"),
          out_dir / "freq_2_mag_sq.png")
    _save(_subplot_fig(draw_xyz,       f"FFT by axis ({mission_label})"),
          out_dir / "freq_3_xyz.png")

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(19, 4))
    fig.suptitle(f"Frequency domain  ({mission_label})", fontsize=13)
    draw_fft(ax1);       ax1.grid(True, lw=0.3)
    draw_welch_psd(ax2); ax2.grid(True, lw=0.3)
    draw_xyz(ax3);       ax3.grid(True, lw=0.3)
    fig.tight_layout()
    _save(fig, out_dir / "freq_combined.png")


def plot_spectrogram(dec_elapsed: np.ndarray, dec_accel: np.ndarray,
                     fs_dec: float, nperseg: int,
                     mission_label: str, out_dir: Path) -> None:
    """Figure 4: spectrogram of decimated accel_z showing wave energy over time."""
    wave_lo, wave_hi = 0.05, 0.50

    def draw_specgram(ax):
        ax.specgram(dec_accel[:, 2], NFFT=nperseg, Fs=fs_dec,
                    noverlap=nperseg // 2, cmap="viridis", scale="dB")
        ax.axhline(wave_lo, lw=0.8, color="white", linestyle="--", label=f"{wave_lo} Hz")
        ax.axhline(wave_hi, lw=0.8, color="white", linestyle=":",  label=f"{wave_hi} Hz")
        ax.set_ylim(0, fs_dec / 2)
        ax.set_xlabel("time  (s)")
        ax.set_ylabel("frequency  (Hz)")
        ax.set_title("Spectrogram — decimated accel_z  (dB)")
        ax.legend(fontsize=8, loc="upper right")

    _save(_subplot_fig(draw_specgram, f"Spectrogram ({mission_label})", figsize=(13, 4)),
          out_dir / "spectrogram.png")


def plot_reference(dec_elapsed: np.ndarray, filt_accel: np.ndarray,
                   eta_time: np.ndarray, eta: np.ndarray,
                   mission_label: str, out_dir: Path) -> None:
    """Figure 5: pipeline accel_z vs Level-2 sea_surface_elevation reference."""
    # TODO(welch): add a second subplot comparing the Welch PSD of filt_accel[:,2]
    # against the PSD of sea_surface_elevation. A spectral comparison is more
    # meaningful than a time-domain overlay since accel and elevation differ by
    # a double integration (factor of (2*pi*f)^4 in power).
    # TODO(wave_metrics): annotate the plot with Hs and Tp from sf_demo_wave_metrics()
    # alongside the microSWIFT Level-2 significant_wave_height and peak_wave_period
    # fields for a direct scalar comparison.
    t_dec = dec_elapsed / 1000.0

    def draw_overlay(ax):
        if eta.size == 0:
            ax.text(0.5, 0.5, "sea_surface_elevation: all NaN",
                    ha="center", va="center", transform=ax.transAxes)
            return
        az_norm  = filt_accel[:, 2]
        eta_norm = eta
        ax2 = ax.twinx()
        ax.plot(t_dec,    az_norm,  lw=0.8, color="steelblue", label="filtered accel_z  (m/s^2)")
        ax2.plot(eta_time, eta_norm, lw=0.8, color="darkorange", alpha=0.8,
                 label="sea_surface_elevation  (m)")
        ax.set_xlabel("time  (s)")
        ax.set_ylabel("accel_z  (m/s^2)", color="steelblue")
        ax2.set_ylabel("sea surface elevation  (m)", color="darkorange")
        ax.set_title("Pipeline output vs Level-2 reference")
        lines1, labels1 = ax.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8)

    _save(_subplot_fig(draw_overlay,
                       f"Reference overlay ({mission_label})", figsize=(13, 4)),
          out_dir / "reference_overlay.png")


def plot_timing(dec_elapsed: np.ndarray, mission_label: str, out_dir: Path) -> None:
    """Figure 6: histogram of decimated sample intervals revealing timing jitter."""
    intervals = np.diff(dec_elapsed.astype(np.float64))
    expected  = float(np.median(intervals))

    def draw_hist(ax):
        ax.hist(intervals, bins=60, color="slategray", edgecolor="white", lw=0.3)
        ax.axvline(expected, color="crimson", lw=1.2,
                   label=f"median  {expected:.1f} ms")
        ax.set_xlabel("sample interval  (ms)")
        ax.set_ylabel("count")
        ax.set_title("Decimated sample intervals  (timing jitter)")
        ax.legend(fontsize=8)

    _save(_subplot_fig(draw_hist,
                       f"Sample intervals ({mission_label})", figsize=(9, 4)),
          out_dir / "timing_histogram.png")


def plot_ahrs(elapsed_ms: np.ndarray, accel_body_z: np.ndarray,
              elapsed_oriented: np.ndarray, accel_oriented: np.ndarray,
              q_oriented: np.ndarray, mission_label: str,
              out_dir: Path) -> None:
    """Figure 3: AHRS — body vs earth frame + Euler angles + individual saves."""
    t_raw = elapsed_ms / 1000.0
    t_ori = elapsed_oriented / 1000.0
    euler = quat_to_euler_deg(q_oriented)

    def draw_body_vs_earth(ax):
        ax.plot(t_raw, accel_body_z, lw=0.5, color="tomato",
                label="body-frame z  (includes gravity ~9.8 m/s^2)")
        ax.plot(t_ori, accel_oriented[:, 2], lw=0.7, color="steelblue",
                label="earth-frame z  (gravity removed)")
        ax.set_ylabel("accel_z  (m/s^2)")
        ax.set_xlabel("time  (s)")
        ax.set_title("Body-frame vs Earth-frame vertical acceleration")
        ax.legend(fontsize=8)

    def draw_euler(ax):
        ax.plot(t_ori, euler[:, 0], lw=0.7, color="firebrick",  label="roll")
        ax.plot(t_ori, euler[:, 1], lw=0.7, color="goldenrod",  label="pitch")
        ax.plot(t_ori, euler[:, 2], lw=0.7, color="mediumblue", label="yaw")
        ax.set_ylabel("angle  (deg)")
        ax.set_xlabel("time  (s)")
        ax.set_title("Buoy orientation (Euler angles from AHRS quaternion)")
        ax.legend(fontsize=8)

    _save(_subplot_fig(draw_body_vs_earth,
                       f"AHRS body vs earth ({mission_label})", figsize=(11, 4)),
          out_dir / "ahrs_1_body_vs_earth.png")
    _save(_subplot_fig(draw_euler,
                       f"AHRS Euler angles ({mission_label})", figsize=(11, 4)),
          out_dir / "ahrs_2_euler.png")

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7))
    fig.suptitle(f"AHRS  ({mission_label})", fontsize=13)
    draw_body_vs_earth(ax1); ax1.grid(True, lw=0.3)
    draw_euler(ax2);          ax2.grid(True, lw=0.3)
    fig.tight_layout()
    _save(fig, out_dir / "ahrs_combined.png")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--mission", type=int, default=34)
    parser.add_argument("--local", type=Path, default=None)
    parser.add_argument("--lib", type=Path, default=None)
    parser.add_argument("--no-plot", action="store_true",
                        help="print stage reports without showing plots")
    args = parser.parse_args()

    out_dir = Path(__file__).parent / "output" / f"mission_{args.mission}"
    out_dir.mkdir(parents=True, exist_ok=True)
    original_print, log_file = tee_prints_to(out_dir / "run_demo.log")
    atexit.register(lambda: (setattr(builtins, "print", original_print),
                             log_file.close()))

    lib     = load_lib(args.lib)
    nperseg = lib.sf_demo_nperseg()

    # Load
    section("Load")
    source = args.local if args.local else args.mission
    mission_label = f"mission {args.mission}" if not args.local else str(args.local.name)
    print(f"  log file   : {out_dir / 'run_demo.log'}")
    print(f"  source     : {source}")
    ds   = load_mission(source)
    buoy = best_trajectory(ds)
    print(f"  trajectory : {int(buoy['trajectory'].values)}"
          f"  (of {ds.sizes['trajectory']})")
    print(f"  imu_freq   : {int(ds['imu_freq'].values[0])} Hz")
    print(f"  nperseg    : {nperseg}  (SMARTFIN_WELCH_NPERSEG)")

    elapsed_ms, accel, gyro, mag = imu_arrays(buoy)
    n_in = len(elapsed_ms)
    print(f"  imu samples: {n_in}")

    # Stage 1: orient_from_imu
    section("Stage 1: orient_from_imu")
    out_elapsed = np.zeros(n_in, dtype=np.uint32)
    out_q       = np.zeros((n_in, 4), dtype=np.float64)
    out_accel   = np.zeros((n_in, 3), dtype=np.float64)
    accel_used  = ctypes.c_int()
    mag_used    = ctypes.c_int()
    bias_used   = ctypes.c_int()

    n_oriented = lib.sf_demo_orient(
        elapsed_ms, accel, gyro, mag, n_in,
        out_elapsed,
        np.ascontiguousarray(out_q),
        np.ascontiguousarray(out_accel),
        ctypes.byref(accel_used),
        ctypes.byref(mag_used),
        ctypes.byref(bias_used),
    )

    elapsed_oriented = out_elapsed[:n_oriented].copy()
    q_oriented       = out_q[:n_oriented].copy()
    accel_oriented   = out_accel[:n_oriented].copy()
    report_accel(elapsed_oriented, accel_oriented)
    report_ahrs_usage(n_oriented, accel_used.value,
                      mag_used.value, bias_used.value)

    # Stage 2: decimate
    section("Stage 2: decimate")
    dec_elapsed = np.zeros(n_oriented, dtype=np.uint32)
    dec_accel   = np.zeros((n_oriented, 3), dtype=np.float64)

    n_dec = lib.sf_demo_decimate(
        elapsed_oriented,
        np.ascontiguousarray(accel_oriented),
        n_oriented,
        dec_elapsed,
        np.ascontiguousarray(dec_accel),
    )

    dec_elapsed = dec_elapsed[:n_dec].copy()
    dec_accel   = dec_accel[:n_dec].copy()
    report_accel(dec_elapsed, dec_accel)

    # Stage 3: bandpass (in-place)
    section("Stage 3: bandpass  (0.05–0.50 Hz)")
    filt_accel = np.ascontiguousarray(dec_accel.copy())
    ret = lib.sf_demo_bandpass(dec_elapsed, filt_accel, n_dec)
    if ret != 0:
        print("  skipped (too few samples or non-positive duration)")
    else:
        report_accel(dec_elapsed, filt_accel)

    # Stage 4: fft
    # TODO(welch): replace manual per-segment FFT loop with a single
    # sf_demo_welch() call that returns WelchResult (freqs, psd, df) for each
    # axis. The Welch estimate averages across all overlapping segments so the
    # single-segment X / mag_sq below becomes unnecessary.
    section(f"Stage 4: fft  (nperseg={nperseg})")
    if n_dec < nperseg:
        print(f"  ERROR: only {n_dec} decimated samples, need {nperseg}")
        sys.exit(1)

    X_pre_all, X_filt_all = [], []
    for axis in range(3):
        pre_ax = np.ascontiguousarray(dec_accel[:nperseg, axis].copy())
        re_p = pre_ax.copy()
        im_p = np.zeros(nperseg, dtype=np.float64)
        lib.sf_demo_fft(re_p, im_p)
        X_pre_all.append(re_p + 1j * im_p)

        filt_ax = np.ascontiguousarray(filt_accel[:nperseg, axis].copy())
        re_f = filt_ax.copy()
        im_f = np.zeros(nperseg, dtype=np.float64)
        lib.sf_demo_fft(re_f, im_f)
        X_filt_all.append(re_f + 1j * im_f)

    X_pre = X_pre_all[2]
    X     = X_filt_all[2]
    seg   = np.ascontiguousarray(filt_accel[:nperseg, 2].copy())
    print(f"  bins      : {nperseg} complex  (3 axes)")
    print(f"  |X_z[0]|  : {abs(X[0]):.4e}  (DC, filtered z)")
    print(f"  |X_z[N/2]|: {abs(X[nperseg // 2]):.4e}  (Nyquist, filtered z)")

    # Stage 5: real_dft_mag_sq
    # TODO(welch): once sf_demo_welch() exists, drop this call and use the
    # averaged PSD from WelchResult instead. real_dft_mag_sq on a single
    # segment is a noisy periodogram; Welch averaging gives a stable estimate.
    section("Stage 5: real_dft_mag_sq")
    mag_sq = np.zeros(nperseg // 2 + 1, dtype=np.float64)
    lib.sf_demo_real_dft_mag_sq(
        np.ascontiguousarray(seg),
        np.ascontiguousarray(mag_sq),
    )

    fs_dec = 1000.0 / float(np.median(np.diff(dec_elapsed.astype(np.float64))))
    df     = fs_dec / nperseg
    freqs  = np.arange(len(mag_sq)) * df
    wave   = (freqs >= 0.05) & (freqs <= 0.50)
    top3   = np.argsort(mag_sq[wave])[-3:][::-1]

    print(f"  one-sided bins : {len(mag_sq)}  (nperseg/2 + 1)")
    print(f"  df             : {df:.4f} Hz")
    # TODO(wave_metrics): replace manual peak search with sf_demo_wave_metrics()
    # output: Hs (significant wave height), Tp (peak period), Tm (mean period).
    print(f"  top peaks in wave band (0.05–0.50 Hz):")
    wf, wm = freqs[wave], mag_sq[wave]
    for i in top3:
        print(f"    {wf[i]:.4f} Hz   |X|^2 = {wm[i]:.4e}")

    if args.no_plot:
        return

    accel_body_z = accel[:, 2].astype(np.float64)

    t0_raw   = buoy["time"].values[0]
    eta_time = ((buoy["time"].values - t0_raw)
                / np.timedelta64(1, "s")).astype(np.float64)
    eta_vals = buoy["sea_surface_elevation"].values.astype(np.float64)
    valid_eta = np.isfinite(eta_vals)
    eta_time  = eta_time[valid_eta]
    eta_vals  = eta_vals[valid_eta]

    plot_pipeline(elapsed_oriented, accel_oriented,
                  dec_elapsed, dec_accel, filt_accel, mission_label, out_dir)

    plot_frequency(X, X_pre, X_filt_all, mag_sq,
                   fs_dec, nperseg, mission_label, out_dir)

    plot_spectrogram(dec_elapsed, dec_accel, fs_dec, nperseg, mission_label, out_dir)

    plot_reference(dec_elapsed, filt_accel, eta_time, eta_vals, mission_label, out_dir)

    plot_timing(dec_elapsed, mission_label, out_dir)

    plot_ahrs(elapsed_ms, accel_body_z,
              elapsed_oriented, accel_oriented, q_oriented, mission_label, out_dir)

    print(f"\n  plots saved to {out_dir}/")


if __name__ == "__main__":
    main()
