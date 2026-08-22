import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution, minimize_scalar


# ============================================================
# Funkce
# ============================================================

def f(I, F, c):
    """
    Zmena intervalu X pro aktualni interval I.

        X = -c*I^2 / (F + c*I)
    """
    return -c * I**2 / (F + c * I)


# ============================================================
# Linearni aproximace jednoho useku
# ============================================================

def linear_fit(x, y):
    """
    Najde linearni aproximaci:

        y ~= a*x + b

    pomoci least-squares.
    """
    A = np.vstack([x, np.ones_like(x)]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return a, b


def segment_error(a, b, x, y):
    """
    Maximalni absolutni chybu aproximace.
    """
    approx = a * x + b
    error = approx - y

    idx = np.argmax(np.abs(error))

    return np.max(np.abs(error)), x[idx], error[idx]


# ============================================================
# Vytvoreni segmentu
# ============================================================

def create_segment(x_min, x_max, F, c, samples=1000):
    """
    Vytvori linearni aproximaci funkce v intervalu.
    """

    x = np.linspace(x_min, x_max, samples)
    y = f(x, F, c)

    a, b = linear_fit(x, y)

    err, err_x, signed_err = segment_error(a, b, x, y)

    return {
        "xmin": x_min,
        "xmax": x_max,
        "a": a,
        "b": b,
        "max_error": err,
        "max_error_x": err_x,
        "max_error_signed": signed_err,
    }


# ============================================================
# Vyhodnoceni celeho rozdeleni
# ============================================================

def evaluate_segments(bounds, F, c, samples_per_segment=1000):
    """
    bounds:
        [I0, I1, I2, ..., In]

    Vytvori n segmentu.
    """

    segments = []

    for i in range(len(bounds) - 1):
        seg = create_segment(
            bounds[i],
            bounds[i + 1],
            F,
            c,
            samples_per_segment
        )

        segments.append(seg)

    return segments


# ============================================================
# Optimalizace hranic
# ============================================================

def optimize_segments(
    I_min,
    I_max,
    F,
    c,
    n,
    samples_per_segment=500
):
    """
    Najde optimalni hranice n linearni segmentu.

    Optimalizacni kriterium:
        minimalizace maximalni absolutni chyby.
    """

    if n == 1:
        bounds = [I_min, I_max]
        return evaluate_segments(
            bounds,
            F,
            c,
            samples_per_segment
        )

    # --------------------------------------------------------
    # Kvuli velkemu rozsahu pouzijeme logaritmicke souradnice.
    #
    # I = exp(t)
    #
    # To je dulezite, protoze interval 200 .. 16 000 000
    # je obrovsky.
    # --------------------------------------------------------

    log_min = np.log(I_min)
    log_max = np.log(I_max)

    def make_bounds(params):
        """
        Prevede optimalizacni parametry na hranice I.
        """

        inner = np.sort(params)

        logs = np.concatenate((
            [log_min],
            inner,
            [log_max]
        ))

        return np.exp(logs)

    # --------------------------------------------------------
    # Cilem je minimalizovat nejvetsi chybu
    # --------------------------------------------------------

    def objective(params):

        bounds = make_bounds(params)

        segments = evaluate_segments(
            bounds,
            F,
            c,
            samples_per_segment
        )

        max_error = max(
            seg["max_error"]
            for seg in segments
        )

        return max_error

    # --------------------------------------------------------
    # Inicialni rozdeleni rovnomerne v log prostoru
    # --------------------------------------------------------

    initial = np.linspace(
        log_min,
        log_max,
        n + 1
    )[1:-1]

    # --------------------------------------------------------
    # Differential evolution
    # --------------------------------------------------------

    result = differential_evolution(
        objective,
        bounds=[(log_min, log_max)] * (n - 1),
        tol=1e-7,
        polish=True,
        seed=1
    )

    final_bounds = make_bounds(result.x)

    return evaluate_segments(
        final_bounds,
        F,
        c,
        samples_per_segment=5000
    )


# ============================================================
# Vypis vysledku
# ============================================================

def print_results(segments, F, c):

    print()
    print("=" * 80)
    print(f"F = {F}")
    print(f"c = {c}")
    print("=" * 80)

    max_error = 0

    for i, seg in enumerate(segments):

        err = seg["max_error"]

        max_error = max(max_error, err)

        print()
        print(f"Segment {i + 1}")
        print("-" * 40)

        print(
            f"I range : "
            f"{seg['xmin']:.6f} .. {seg['xmax']:.6f}"
        )

        print(
            f"X ~= a*I + b"
        )

        print(
            f"a = {seg['a']:.15e}"
        )

        print(
            f"b = {seg['b']:.15e}"
        )

        print(
            f"max absolute error = {err:.9f}"
        )

        print(
            f"error at I = {seg['max_error_x']:.6f}"
        )

        # Relativni chyba
        exact = f(seg["max_error_x"], F, c)
        rel = abs(err / exact) * 100

        print(
            f"relative error there = {rel:.6f} %"
        )

    print()
    print("=" * 80)
    print(
        f"GLOBAL MAX ERROR = {max_error:.9f}"
    )
    print("=" * 80)


# ============================================================
# Graf
# ============================================================

def plot_result(
    segments,
    F,
    c,
    I_min,
    I_max
):

    x = np.geomspace(I_min, I_max, 10000)

    y_exact = f(x, F, c)

    y_approx = np.zeros_like(x)

    for seg in segments:

        mask = (
            (x >= seg["xmin"]) &
            (x <= seg["xmax"])
        )

        y_approx[mask] = (
            seg["a"] * x[mask] +
            seg["b"]
        )

    # --------------------------------------------------------
    # Funkce
    # --------------------------------------------------------

    plt.figure(figsize=(10, 6))

    plt.plot(
        x,
        y_exact,
        label="exact"
    )

    plt.plot(
        x,
        y_approx,
        "--",
        label="piecewise linear"
    )

    for seg in segments:
        plt.axvline(
            seg["xmin"],
            linestyle=":"
        )

    plt.xscale("log")

    plt.xlabel("I")
    plt.ylabel("X")

    plt.title(
        f"Piecewise linear approximation, c={c}"
    )

    plt.grid(True)
    plt.legend()

    plt.show()

    # --------------------------------------------------------
    # Chyba
    # --------------------------------------------------------

    error = np.abs(y_exact - y_approx)

    plt.figure(figsize=(10, 6))

    plt.plot(
        x,
        error
    )

    plt.xscale("log")

    plt.xlabel("I")
    plt.ylabel("|error|")

    plt.title("Absolute error")

    plt.grid(True)

    plt.show()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    # --------------------------------------------------------
    # PARAMETRY
    # --------------------------------------------------------

    F = 16_000_000

    c = 100

    I_min = 200
    I_max = 16_000_000

    n = 6

    # --------------------------------------------------------
    # Optimalizace
    # --------------------------------------------------------

    segments = optimize_segments(
        I_min=I_min,
        I_max=I_max,
        F=F,
        c=c,
        n=n
    )

    # --------------------------------------------------------
    # Vysledky
    # --------------------------------------------------------

    print_results(
        segments,
        F,
        c
    )

    # --------------------------------------------------------
    # Graf
    # --------------------------------------------------------

    plot_result(
        segments,
        F,
        c,
        I_min,
        I_max
    )
