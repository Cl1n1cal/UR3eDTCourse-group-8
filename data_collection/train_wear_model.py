import sys, pickle, json, warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime

from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from sklearn.linear_model import LinearRegression
from sklearn.ensemble import RandomForestRegressor
from sklearn.neural_network import MLPRegressor
from sklearn.metrics import r2_score, mean_absolute_error

try:
    from xgboost import XGBRegressor
    XGBOOST = True
except ImportError:
    XGBOOST = False
    print("[warn] xgboost not installed — skipping")

warnings.filterwarnings("ignore")

SCRIPT_DIR   = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent

if len(sys.argv) > 1:
    DATASET_PATH = Path(sys.argv[1])
else:
    DATASET_PATH = SCRIPT_DIR / "wear_dataset.csv"

OUTPUT_DIR = PROJECT_ROOT / "ml_models"
MODEL_PATH = OUTPUT_DIR / "wear_model.pkl"
META_PATH = OUTPUT_DIR / "wear_model_metadata.json"
FIG_COMPARISON = SCRIPT_DIR / "fig_model_comparison.png"
FIG_DIAGNOSTICS = SCRIPT_DIR / "fig_model_diagnostics.png"

FEATURE_NAMES = [
    "mean_dx", "mean_dy", "mean_dz",
    "max_dx",  "max_dy",  "max_dz",
    "mean_drx","mean_dry","mean_drz",
    "max_drx", "max_dry", "max_drz",
    "mean_pos_error_m",
    "max_pos_error_m",
]
TARGET = "wear_level"
TEST_SIZE = 0.2
RANDOM_STATE = 42
TWO_PI = 2.0 * np.pi


def load_and_clean(path: Path):
    df = pd.read_csv(path)
    print(f"Loaded {len(df)} rows from {path.name}")

    missing = [f for f in FEATURE_NAMES if f not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}")

    df = df.dropna(subset=FEATURE_NAMES + [TARGET])

    for col in [c for c in FEATURE_NAMES if "dr" in c]:
        a = df[col].abs()
        df[col] = np.minimum(a, TWO_PI - a)

    mask = pd.Series(True, index=df.index)
    for col in FEATURE_NAMES:
        q1, q3 = df[col].quantile(0.01), df[col].quantile(0.99)
        iqr = q3 - q1
        mask &= df[col].between(q1 - 4*iqr, q3 + 4*iqr)
    removed = (~mask).sum()
    if removed:
        print(f"  Removed {removed} outlier rows")
    df = df[mask].reset_index(drop=True)
    print(f"  Clean rows: {len(df)}")

    X = df[FEATURE_NAMES].values.astype(np.float64)
    y = df[TARGET].values.astype(np.float64)
    return X, y, df


def make_models():
    models = {
        "Random Forest": Pipeline([
            ("sc", StandardScaler()),
            ("rf", RandomForestRegressor(
                n_estimators=400, max_depth=6,
                min_samples_leaf=3, random_state=RANDOM_STATE, n_jobs=-1)),
        ]),
        "DNN (MLP)": Pipeline([
            ("sc", StandardScaler()),
            ("mlp", MLPRegressor(
                hidden_layer_sizes=(256, 128, 64), activation="relu",
                alpha=0.01, learning_rate_init=0.001,
                max_iter=1000, early_stopping=True,
                validation_fraction=0.1, n_iter_no_change=20,
                random_state=RANDOM_STATE)),
        ]),
        "Linear Regression": Pipeline([
            ("sc", StandardScaler()),
            ("lr", LinearRegression()),
        ]),
    }
    if XGBOOST:
        models["XGBoost"] = Pipeline([
            ("sc", StandardScaler()),
            ("xgb", XGBRegressor(
                n_estimators=200, max_depth=5, learning_rate=0.03,
                colsample_bytree=0.8, subsample=0.8,
                random_state=RANDOM_STATE, verbosity=0, n_jobs=-1)),
        ])
    return models


def plot_comparison(results: dict, y_test: np.ndarray, path: Path):
    ordered = sorted(results.items(), key=lambda x: -x[1]["r2_te"])
    n = len(ordered)

    fig, axes = plt.subplots(1, n, figsize=(4.2 * n, 4.2), constrained_layout=True)
    if n == 1:
        axes = [axes]

    for ax, (name, res) in zip(axes, ordered):
        y_pred = np.clip(res["y_pred"], 0.0, 1.0)
        ax.scatter(y_test, y_pred, alpha=0.35, s=14, color="#3A86FF", edgecolors="none")
        ax.plot([0, 1], [0, 1], "r--", lw=1.2, label="Perfect")
        ax.set_xlim(-0.05, 1.05)
        ax.set_ylim(-0.05, 1.05)
        ax.set_xlabel("Actual wear level", fontsize=10)
        ax.set_ylabel("Predicted wear level", fontsize=10)
        ax.set_title(
            f"{name}\n$R^2_{{train}}$={res['r2_tr']:.3f}   "
            f"$R^2_{{test}}$={res['r2_te']:.3f}   MAE={res['mae']:.3f}",
            fontsize=9,
        )
        ax.grid(alpha=0.25)

    fig.suptitle("Wear Model Benchmark — Actual vs Predicted (test set)", fontsize=12)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Figure 14: {path.name}")


def plot_diagnostics(best_name: str, results: dict, X_test: np.ndarray, y_test: np.ndarray, df_full: pd.DataFrame, path: Path):
    res   = results[best_name]
    model = res["model"]
    y_pred = np.clip(res["y_pred"], 0.0, 1.0)
    resid  = y_test - y_pred

    fig = plt.figure(figsize=(18, 11), constrained_layout=True)
    fig.suptitle(f"Wear Prediction Model Diagnostics  —  {best_name}", fontsize=14, fontweight="bold")
    gs  = fig.add_gridspec(2, 3)
    ax1 = fig.add_subplot(gs[0, 0])
    ax2 = fig.add_subplot(gs[0, 1])
    ax3 = fig.add_subplot(gs[0, 2])
    ax4 = fig.add_subplot(gs[1, 0])
    ax5 = fig.add_subplot(gs[1, 1])
    ax6 = fig.add_subplot(gs[1, 2])

    # panel a
    ordered = sorted(results.items(), key=lambda x: -x[1]["r2_te"])
    names = [k for k, _ in ordered]
    r2_tr = [v["r2_tr"] for _, v in ordered]
    r2_te = [v["r2_te"] for _, v in ordered]
    x_pos = np.arange(len(names))
    width = 0.35
    bars1 = ax1.bar(x_pos - width/2, r2_tr, width, label="Train", color="#3A86FF", alpha=0.8)
    bars2 = ax1.bar(x_pos + width/2, r2_te, width, label="Test",  color="#FF6B6B", alpha=0.8)
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels([n.replace(" ", "\n") for n in names], fontsize=8)
    ax1.set_ylabel("$R^2$")
    ax1.set_ylim(0.80, 1.01)
    ax1.set_title("(a) Model Comparison — $R^2$")
    ax1.legend(fontsize=8)
    ax1.grid(axis="y", alpha=0.3)
    for b in list(bars1) + list(bars2):
        ax1.text(b.get_x() + b.get_width()/2, b.get_height() + 0.002, f"{b.get_height():.3f}", ha="center", va="bottom", fontsize=7)

    # panel b
    for step_name, step in model.steps:
        if hasattr(step, "feature_importances_"):
            imp = step.feature_importances_
            break
    else:
        imp = None

    if imp is not None:
        idx = np.argsort(imp)
        ax2.barh([FEATURE_NAMES[i] for i in idx], imp[idx], color="steelblue", edgecolor="white")
        ax2.set_xlabel("Importance")
        ax2.set_title(f"(b) Feature Importance ({best_name})")
    else:
        ax2.text(0.5, 0.5, "Not available", ha="center", va="center", transform=ax2.transAxes)
        ax2.set_title("(b) Feature Importance")

    # panel c
    ax3.scatter(y_test, y_pred, alpha=0.3, s=10, color="steelblue")
    ax3.plot([0, 1], [0, 1], "r--", lw=1.5, label="Perfect")
    ax3.set_xlabel("Actual wear level")
    ax3.set_ylabel("Predicted wear level")
    ax3.set_title(
        f"(c) Actual vs Predicted — {best_name}\n"
        f"$R^2$={res['r2_te']:.3f}   MAE={res['mae']:.3f}"
    )
    ax3.legend(fontsize=8)
    ax3.grid(alpha=0.25)

    # panel d
    ax4.scatter(y_pred, resid, alpha=0.3, s=10, color="coral")
    ax4.axhline(0, color="black", lw=1.2)
    ax4.set_xlabel("Predicted wear level")
    ax4.set_ylabel("Residual  (actual - predicted)")
    ax4.set_title("(d) Residual Plot")
    ax4.grid(alpha=0.25)

    # panel e
    abs_err = np.abs(resid)
    ax5.hist(abs_err, bins=40, color="mediumseagreen", edgecolor="white", lw=0.4)
    ax5.axvline(abs_err.mean(), color="red", lw=1.5, label=f"Mean={abs_err.mean():.3f}")
    ax5.axvline(np.percentile(abs_err,90),color="orange", lw=1.5, label=f"P90={np.percentile(abs_err,90):.3f}")
    ax5.set_xlabel("|Residual|")
    ax5.set_ylabel("Count")
    ax5.set_title("(e) Absolute Error Distribution")
    ax5.legend(fontsize=8)

    # panel f
    grouped = df_full.groupby("wear_level")["mean_pos_error_m"]
    wl_vals = sorted(df_full["wear_level"].unique())
    means = [grouped.get_group(w).mean() for w in wl_vals]
    stds = [grouped.get_group(w).std() for w in wl_vals]
    ax6.errorbar(wl_vals, means, yerr=stds, fmt="o-", capsize=4, color="steelblue", lw=1.5)
    ax6.set_xlabel("Injected wear level")
    ax6.set_ylabel("Mean TCP position error (m)")
    ax6.set_title("(f) TCP Discrepancy vs Wear Level")
    ax6.grid(alpha=0.3)

    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Figure 15: {path.name}")


def main():
    print("=" * 62)
    print("Wear Model Training")
    print(f"Dataset : {DATASET_PATH}")
    print("=" * 62)

    X, y, df_full = load_and_clean(DATASET_PATH)

    print(f"\nSamples : {len(X)}")
    print("Wear level distribution:")
    for wl in sorted(set(y)):
        print(f"  {wl:.1f} : {(y == wl).sum()} samples")

    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=TEST_SIZE, random_state=RANDOM_STATE)
    print(f"\nTrain: {len(X_train)} | Test: {len(X_test)}\n")

    models = make_models()
    results = {}

    print(f"{'Model':<22} {'R²_train':>10} {'R²_test':>10} {'MAE':>8}")
    print("-" * 54)
    for name, m in models.items():
        m.fit(X_train, y_train)
        r2_tr = r2_score(y_train, m.predict(X_train))
        y_pred = m.predict(X_test)
        r2_te = r2_score(y_test, y_pred)
        mae = mean_absolute_error(y_test, y_pred)
        print(f"{name:<22} {r2_tr:>10.3f} {r2_te:>10.3f} {mae:>8.3f}")
        results[name] = {"model": m, "r2_tr": r2_tr, "r2_te": r2_te, "mae": mae, "y_pred": y_pred}

    best_name = max(results, key=lambda k: results[k]["r2_te"])
    best = results[best_name]

    print(f"\nGenerating figures …")
    plot_comparison(results, y_test, FIG_COMPARISON)
    plot_diagnostics(best_name, results, X_test, y_test, df_full, FIG_DIAGNOSTICS)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    with open(MODEL_PATH, "wb") as f:
        pickle.dump(best["model"], f)

    meta = {
        "trained_at" : datetime.now().isoformat(),
        "dataset" : str(DATASET_PATH),
        "n_samples" : len(X),
        "n_features" : len(FEATURE_NAMES),
        "feature_names": FEATURE_NAMES,
        "split" : f"{int((1-TEST_SIZE)*100)}/{int(TEST_SIZE*100)} train/test",
        "random_state" : RANDOM_STATE,
        "best_model" : best_name,
        "r2_train" : round(float(best["r2_tr"]), 4),
        "r2_test" : round(float(best["r2_te"]), 4),
        "mae_test" : round(float(best["mae"]), 4),
        "all_models" : {
            n: {"r2_train": round(float(v["r2_tr"]), 4),
                "r2_test": round(float(v["r2_te"]), 4),
                "mae_test": round(float(v["mae"]), 4)}
            for n, v in results.items()
        },
    }
    with open(META_PATH, "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nModel: {MODEL_PATH.name}")
    print(f"Metadata: {META_PATH.name}")
    print(f"\nBest model: {best_name}")
    print(f"R²_train: {best['r2_tr']:.3f}")
    print(f"R²_test: {best['r2_te']:.3f}")
    print(f"MAE: {best['mae']:.3f} (±{best['mae']:.3f} wear units on average)")


if __name__ == "__main__":
    main()
